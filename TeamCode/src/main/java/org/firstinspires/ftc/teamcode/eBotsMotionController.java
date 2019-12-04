package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.eBotsAuton2019.Speed;

import java.util.Arrays;

import static java.lang.String.format;
import static org.firstinspires.ftc.teamcode.eBotsAuton2019.*;

public class eBotsMotionController {
    /**
     * This class functions as a PID controller for translation and rotation
     * using a 3 encoder setup
     */

    /*****************************************************************
    //******    CLASS VARIABLES
    //***************************************************************/

    /*****************************************************************
    //******    ENUMERATIONS
    //***************************************************************/


    /*****************************************************************
    //******    CONSTRUCTORS
    //***************************************************************/


    /*****************************************************************
     //******    STATIC METHODS
     //***************************************************************/
    public static void moveToTargetPose(TrackingPose trackingPose, Speed speed, GyroSetting gyroSetting, Accuracy accuracy, SoftStart softStart, BNO055IMU imu, Telemetry telemetry){
        /*
        1) Calculate x,y, spin components of error
        2) Calculate direction for travel
        3) Calculate x,y, spin components for Raw Drive intent (raw because can exceed motor limits)
            a) if hyp of x,y is saturated, must scale the drive intent for each vector
        4) Calculate motor drive vectors based on raw drive intent (these get scaled automatically for equipment limitations)
        5) Scale drive vectors based on Speed settings
        6) Continue looping until exit conditions occur
         */

        final boolean debugOn = true;
        final String logTag = "BTI_eBotsMotionCont";
        double debugIncrement = 1.0;

        if (debugOn) {
            Log.d(logTag, "Entering eBotsMotionController");
            Log.d(logTag, "Start Position " + trackingPose.toString());
            Log.d(logTag, "Target Position " + trackingPose.getTargetPose().toString());
            Log.d(logTag, "Error " + trackingPose.printError());
            Log.d(logTag, "Speed Settings " + speed.toString());
            Log.d(logTag, "Accuracy Settings " + accuracy.toString());
        }

        trackingPose.resetErrorSum();       //Zero out integrator errorSums for X, Y, and spin

        //Read in the key drive components
        double xError;
        double yError;
        double spinError;

        double xErrorSum;
        double yErrorSum;
        double spinErrorSum;

        //Prepare loop variables
        double xRawSig;
        double yRawSig;
        double spinRawSig;

        boolean useVirtualEncoders = (getDriveMotors().size() == 0) ? true: false;
        boolean translateIntegratorOn = (speed.getK_i()==0.0) ? false : true;
        boolean spinIntegratorOn = (speed.getS_i()==0.0) ? false : true;

        StopWatch travelLegTimer = new StopWatch();
        long timeLimit = 7000L;
        long loopStartTime;
        long loopEndTime;
        long loopDuration;

        int loopCount = 0;

        boolean timedOut = (travelLegTimer.getElapsedTimeMillis() < timeLimit) ? false: true;
        boolean isTargetPoseReached = checkTravelExitConditions(trackingPose, accuracy);

        while(!isTargetPoseReached && !timedOut) {
            loopCount++;
            loopStartTime = travelLegTimer.getElapsedTimeMillis();
            if (debugOn) Log.d(logTag, "____________Entering Loop " + travelLegTimer.toString(loopCount) + "_________________");

            //Log ever loop on increment
            if (debugOn) logPosition(trackingPose, loopCount, travelLegTimer);

            if (loopCount != 1) EncoderTracker.updatePoseUsingThreeEncoders(trackingPose, imu);
            trackingPose.calculatePoseError();  //Refresh error values

            xError = trackingPose.getXError();
            yError = trackingPose.getYError();
            spinError = trackingPose.getSpinError();

            xErrorSum = trackingPose.getXErrorSum();
            yErrorSum = trackingPose.getYErrorSum();
            spinErrorSum = trackingPose.getSpinErrorSum();

            xRawSig = xError * speed.getK_p() + xErrorSum * speed.getK_i();
            yRawSig = yError * speed.getK_p() + yErrorSum * speed.getK_i();
            spinRawSig = spinError * speed.getS_p() + spinErrorSum * speed.getS_i();

            if (debugOn) Log.d(logTag, "Raw Signals X / Y / Spin: " + format("%.2f", xRawSig)
                    + " / " + format("%.2f", yRawSig) + " / " + format("%.2f", spinRawSig));
            //  hypotenuse of the raw translate signal
            double translateRawMagnitude = Math.hypot(xRawSig, yRawSig);

            //Now add the integrator if the following conditions are met
            // 1) The integrator is on
            // 2) Either a) raw signal not saturated or b) error is different sign than error sum
            boolean isXRawSignalSaturated = (Math.abs(xRawSig) < 1) ? false : true;
            boolean isYRawSignalSaturated = (Math.abs(yRawSig) < 1) ? false : true;
            boolean isSpinRawSignalSaturated = (Math.abs(spinRawSig) < 1) ? false : true;

            boolean isXErrorSignDifferent = isSignDifferent(xError, xErrorSum);
            boolean isYErrorSignDifferent = isSignDifferent(yError, yErrorSum);
            boolean isSpinErrorSignDifferent = isSignDifferent(spinError, spinErrorSum);

            //  End the control loop here because loopDuration is needed for the Integral term
            loopEndTime = travelLegTimer.getElapsedTimeMillis();
            loopDuration = loopEndTime - loopStartTime;

            if (translateIntegratorOn && (!isXRawSignalSaturated | isXErrorSignDifferent)) trackingPose.addToXErrorSum(xError * (loopDuration / 1000.0));
            if (translateIntegratorOn && (!isYRawSignalSaturated | isYErrorSignDifferent)) trackingPose.addToYErrorSum(yError * (loopDuration / 1000.0));
            if (spinIntegratorOn && (!isSpinRawSignalSaturated | isSpinErrorSignDifferent))trackingPose.addToSpinErrorSum(spinError * (loopDuration / 1000.0));


            //Now energize the motors or cycle virtual encoders
            if (debugOn) Log.d(logTag, "useVirtualEncoders: " + useVirtualEncoders);
            if (debugOn) Log.d(logTag, "driveMotorSize: " + getDriveMotors().size());
            if (!useVirtualEncoders) {
                activateDriveMotors(trackingPose, xRawSig, yRawSig, spinRawSig, speed,softStart ,travelLegTimer, telemetry);
            } else {
                //The encoders use the travelDirection to determine changes to the x and y coordinates
                trackingPose.updateTravelDirection(xRawSig, yRawSig);
                //  Now figure out the output proportions for translate and spin
                double translateSignalOut = (translateRawMagnitude < 1) ? translateRawMagnitude:1;
                translateSignalOut *= speed.getMaxSpeed();

                //  This may need to be backed down if the spin pushes it over 1
                double spinSignalOut = (Math.abs(spinRawSig) < speed.getTurnSpeed()) ? spinRawSig : (speed.getTurnSpeed() * Math.signum(spinRawSig));

                //  Reduce translateSignal by spin amount
                if ((Math.abs(translateSignalOut) + Math.abs(spinSignalOut)) > 1.0){
                    translateSignalOut = 1.0 - Math.abs(spinSignalOut);
                }

                double travelDirection = Math.toDegrees(Math.atan2(yRawSig, xRawSig));
                if(debugOn) Log.d(logTag, "Travel Direction: @" + format("%.2f", travelDirection));

                if (getDriveMotors().size() == 0){
                    EncoderTracker.updateVirtualEncoders(translateSignalOut, travelDirection, spinSignalOut, loopDuration, trackingPose);
                }
            }

            loopStartTime = loopEndTime;

            //TODO:  Update to get access to gyro, currently is instance method, need static caller
            // this is needed to refine odometry angle calculations -- maybe call back to AutonOpMode
            if (gyroSetting.isGyroOn() && (loopCount % gyroSetting.getReadFrequency() == 0)) {
                if (debugOn) {
                    if (imu != null) {
                        double gyroHeadingReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                        double headingFromGyro = gyroHeadingReading + trackingPose.getInitialGyroOffset();
                        double odometryHeadingError = trackingPose.getHeading() - headingFromGyro;
                        Log.d(logTag, "Gyro Reading: " + format("%.2f", headingFromGyro) +
                                " , Heading from Odometry: " + format("%.2f", trackingPose.getHeading()) +
                                " , Error in Odometry Heading: " + format("%.2f", odometryHeadingError));
                        Log.d(logTag, "Gyro Reading: " + format("%.2f", gyroHeadingReading + trackingPose.getInitialGyroOffset()));
                    }
                    //Log.d(logTag, "About to set heading from gyro...");
                }
                //trackingPose.setHeadingFromGyro(eBotsOpMode2019.getGyroReadingDegrees());  //Update heading with robot orientation
            }




            timedOut = (travelLegTimer.getElapsedTimeMillis() < timeLimit) ? false: true;
            isTargetPoseReached = checkTravelExitConditions(trackingPose, accuracy);
            if (debugOn) Log.d(logTag, "____________End Loop " + loopCount + "_________________");

        }

        if (debugOn) {
            if (isTargetPoseReached) {
                Log.d(logTag, "Pose Achieved in " + format("%.2f", travelLegTimer.getElapsedTimeSeconds()));
            } else {
                Log.d(logTag, "Failed to reach target, timed out!!! " + trackingPose.printError());
            }
        }

        stopMotors();
    }

    public static void activateDriveMotors(TrackingPose trackingPose, double xSig, double ySig, double spinSig, Speed speed, SoftStart softStart, StopWatch travelLegTimer, Telemetry telemetry) {
        boolean debugOn = true;
        String logTag = "BTI_activateDriveMotors";
        if (debugOn) Log.d(logTag, "Entering activateDriveMotors");

        //double travelDirectionRad = trackingPose.getHeadingErrorRad();
        double travelDirectionRad = Math.atan2(ySig, xSig);
        double translateSig = Math.hypot(xSig, ySig);
        double robotHeadingRad = trackingPose.getHeadingRad();
        double robotAngle = travelDirectionRad - robotHeadingRad + Math.PI / 4;

        //  Calculate the driveValues, ignoring spin for first pass
        double[] driveValues = new double[4];

        driveValues[0] = Math.cos(robotAngle) * translateSig;
        driveValues[1] = Math.sin(robotAngle) * translateSig;
        driveValues[2] = Math.sin(robotAngle) * translateSig;
        driveValues[3] = Math.cos(robotAngle) * translateSig;

        if (debugOn)
            Log.d(logTag, "Drive vector, no spin or scaling: " + Arrays.toString(driveValues));

        //Now capture the max drive value from the array
        double maxDriveValue = findMaxAbsValue(driveValues);
        double maxSpeedSetting = speed.getMaxSpeed();
        //  Compare the max drive value to the speed setting, select the lower of the two for the scale factor
        double maxAllowedAbsValue = (maxDriveValue < maxSpeedSetting) ? maxDriveValue : maxSpeedSetting;
        //  Scale the drive values
        double scaleFactor = (maxDriveValue == 0.0) ? 0.0 : maxAllowedAbsValue / maxDriveValue;
        scaleDrive(scaleFactor, driveValues);
        if (debugOn)
            Log.d(logTag, "Drive vector, no spin after scaling: " + Arrays.toString(driveValues));
        //Now compare the spin signal to the speed setting
        double conditionedSpinSignal = (speed.getTurnSpeed() < Math.abs(spinSig)) ? (speed.getTurnSpeed() * Math.signum(spinSig)) : spinSig;

        //Note, now that spin is field orientated, indices 0&2 are negative while 1&3 are positive
        driveValues[0] -= conditionedSpinSignal;
        driveValues[1] += conditionedSpinSignal;
        driveValues[2] -= conditionedSpinSignal;
        driveValues[3] += conditionedSpinSignal;

        if (debugOn)
            Log.d(logTag, "Drive vector, with spin no scaling: " + Arrays.toString(driveValues));

        //Now scale the drive signals to makes sure:
        // a) no signal is greater than 1
        // or b) no signal exceeds softStart criteria
        maxDriveValue = findMaxAbsValue(driveValues);
        maxAllowedAbsValue = (maxDriveValue < 1.0) ? maxDriveValue : 1.0;

        //Applies softStart if applicable (routine checks softStart to make sure active and relevant)
        maxAllowedAbsValue = getSoftStartScalingFactor(softStart, maxAllowedAbsValue, travelLegTimer);
        scaleFactor = (maxDriveValue == 0.0) ? 0.0 : maxAllowedAbsValue / maxDriveValue;

        scaleDrive(scaleFactor, driveValues);
        if (debugOn)
            Log.d(logTag, "Output Drive vector, with spin and scaling (inc softStart): " + Arrays.toString(driveValues) +
                    " elapsed time: " + travelLegTimer.toString());

        writeMoveTelemetry(telemetry, driveValues, trackingPose);

        //  Finally, apply the calculated drive vectors
        int i = 0;
        for (DcMotor m : getDriveMotors()) {
            m.setPower(driveValues[i]);
            i++;
        }
    }

    private static boolean checkTravelExitConditions(TrackingPose trackingPose, Accuracy accuracy){
        boolean debugOn = true;
        String logTag  = "BTI_checkTravelExit";

        double spinTolerance = accuracy.getHeadingAngleAccuracy();
        double positionTolerance = accuracy.getPositionalAccuracy();
        double integratorUnwindTolerance = accuracy.getIntegratorUnwindLimit();
        double spinIntegratorUnwindTolerance = accuracy.getSpinIntegratorUnwindLimit();

        boolean xPositionReached = (Math.abs(trackingPose.getXError()) > positionTolerance) ? false: true;
        boolean yPositionReached = (Math.abs(trackingPose.getYError()) > positionTolerance) ? false: true;
        boolean spinTargetReached = (Math.abs(trackingPose.getSpinError()) > spinTolerance) ? false : true;
        boolean xIntegratorUnwound = (Math.abs(trackingPose.getXErrorSum()) > integratorUnwindTolerance) ? false : true;
        boolean yIntegratorUnwound = (Math.abs(trackingPose.getYErrorSum()) > integratorUnwindTolerance) ? false : true;
        boolean spinIntegratorUnwound = (Math.abs(trackingPose.getSpinErrorSum()) > spinIntegratorUnwindTolerance) ? false : true;

        if(debugOn) {
            String results = "xPos: " + xPositionReached + ", yPos: " + yPositionReached + ", spin: " + spinTargetReached;
            results = results + ", xInt: " + xIntegratorUnwound + ", yInt: " + yIntegratorUnwound + ", spinInt: " + spinIntegratorUnwound;
            Log.d(logTag, results);
        }

        if (xPositionReached && yPositionReached && spinTargetReached
                && xIntegratorUnwound && yIntegratorUnwound && spinIntegratorUnwound){
            return true;
        } else {
            return false;
        }
    }

    private static boolean isSignDifferent(double errorValue, double errorSumValue){
        boolean debugOn = false;
        String logTag = "BTI_isSignDifferent";
        boolean returnValue;
        if (errorSumValue == 0.0 | (Math.signum(errorValue) == Math.signum(errorSumValue))) {
            returnValue = false;
        } else {
            returnValue = true;
        }
        return returnValue;
    }
    private static void logPosition(TrackingPose trackingPose, int loopCount, StopWatch timer){
        String logTag = "BTI_logPosition";

        Log.d(logTag, timer.toString(loopCount));
        Log.d(logTag, "Start Position " + trackingPose.toString());
        Log.d(logTag, "Target Position " + trackingPose.getTargetPose().toString());
        Log.d(logTag, "Error " + trackingPose.printError());

    }

    private static void writeMoveTelemetry(Telemetry telemetry, double[] driveValues, TrackingPose trackingPose){
        telemetry.clear();
        telemetry.addData("Motors Found", getDriveMotors().size());
        telemetry.addData("Drive Values", formatArray(driveValues));
        telemetry.addLine(trackingPose.toString());
        telemetry.addLine(trackingPose.getTargetPose().toString());
        telemetry.addLine(trackingPose.printError());
        telemetry.update();
    }

    private static String formatArray(double[] driveValues){
        String out = "";
        for(int i=0; i<driveValues.length; i++){
            if (i>0) out = out + ", ";
            out = out + "[" + format("%.2f", driveValues[i]) + "]";
        }
        return out;
    }

}
