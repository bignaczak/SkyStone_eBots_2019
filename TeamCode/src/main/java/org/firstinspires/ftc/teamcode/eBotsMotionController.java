package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.eBotsAuton2019.Speed;

import java.util.ArrayList;
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
    public static void moveToTargetPose(TrackingPose trackingPose, Speed speed, GyroSetting gyroSetting, Accuracy accuracy){
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

        StopWatch timer = new StopWatch();
        long timeLimit = 5000L;
        long loopStartTime = timer.getElapsedTimeMillis();
        long loopEndTime;
        long loopDuration;

        int loopCount = 0;

        boolean timedOut = (timer.getElapsedTimeMillis() < timeLimit) ? false: true;
        boolean isTargetPoseReached = checkTravelExitConditions(trackingPose, accuracy);

        while(!isTargetPoseReached && !timedOut) {
            loopCount++;
            loopStartTime = timer.getElapsedTimeMillis();
            if (debugOn) Log.d(logTag, "____________Entering Loop " + timer.toString(loopCount) + "_________________");

            //Log ever loop on increment
            if (debugOn) logPosition(trackingPose, loopCount, timer);

            if (loopCount != 1) EncoderTracker.updatePoseUsingThreeEncoders(trackingPose);
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
            loopEndTime = timer.getElapsedTimeMillis();
            loopDuration = loopEndTime - loopStartTime;

            if (translateIntegratorOn && (!isXRawSignalSaturated | isXErrorSignDifferent)) trackingPose.addToXErrorSum(xError * (loopDuration / 1000.0));
            if (translateIntegratorOn && (!isYRawSignalSaturated | isYErrorSignDifferent)) trackingPose.addToYErrorSum(yError * (loopDuration / 1000.0));
            if (spinIntegratorOn && (!isSpinRawSignalSaturated | isSpinErrorSignDifferent))trackingPose.addToSpinErrorSum(spinError * (loopDuration / 1000.0));


            //Now energize the motors or cycle virtual encoders
            if (!useVirtualEncoders) {
                activateDriveMotors(trackingPose, xRawSig, yRawSig, spinRawSig, speed);
            } else {
                //The encoders use the travelDirection to determine changes to the x and y coordinates
                trackingPose.updateTravelDirection();
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

                EncoderTracker.updateVirtualEncoders(translateSignalOut, travelDirection, spinSignalOut, loopDuration, trackingPose);
            }

            loopStartTime = loopEndTime;

            //TODO:  Update to get access to gyro, currently is instance method, need static caller
            // this is needed to refine odometry angle calculations -- maybe call back to AutonOpMode
            if (gyroSetting.isGyroOn() && (loopCount % gyroSetting.getReadFrequency() == 0)) {
                if (debugOn) Log.d(logTag, "About to set heading from gyro...");
                //trackingPose.setHeadingFromGyro(eBotsOpMode2019.getGyroReadingDegrees());  //Update heading with robot orientation
            }




            timedOut = (timer.getElapsedTimeMillis() < timeLimit) ? false: true;
            isTargetPoseReached = checkTravelExitConditions(trackingPose, accuracy);
            if (debugOn) Log.d(logTag, "____________End Loop " + loopCount + "_________________");

        }

        if (debugOn) {
            if (isTargetPoseReached) {
                Log.d(logTag, "Pose Achieved in " + format("%.2f", timer.getElapsedTimeSeconds()));
            } else {
                Log.d(logTag, "Failed to reach target, timed out!!! " + trackingPose.printError());
            }
        }

        stopMotors();
    }

    public static void activateDriveMotors(TrackingPose trackingPose, double xSig, double ySig, double spinSig, Speed speed){
        boolean debugOn = true;
        String logTag = "BTI_activateDriveMotors";
        if (debugOn) Log.d(logTag, "Entering activateDriveMotors");

        //double travelDirectionRad = trackingPose.getHeadingErrorRad();
        double travelDirectionRad = Math.atan2(ySig, xSig);
        double translateSig = Math.hypot(xSig, ySig);
        double robotHeadingRad = trackingPose.getHeadingRad();
        double robotAngle = travelDirectionRad - robotHeadingRad + Math.PI/4;

        //  Calculate the driveValues, ignoring spin for first pass
        double[] driveValues = new double[4];

        driveValues[0] = Math.cos(robotAngle) * translateSig;
        driveValues[1] = Math.sin(robotAngle) * translateSig;
        driveValues[2] = Math.sin(robotAngle) * translateSig;
        driveValues[3] = Math.cos(robotAngle) * translateSig;

        if (debugOn) Log.d(logTag, "Drive vector, no spin or scaling: " + Arrays.toString(driveValues));

        //Now capture the max drive value from the array
        double maxDriveValue = findMaxAbsValue(driveValues);
        double maxSpeedSetting = speed.getMaxSpeed();
        //  Compare the max drive value to the speed setting, select the lower of the two for the scale factor
        double appliedScale = (maxDriveValue < maxSpeedSetting) ? maxDriveValue : maxSpeedSetting;
        //  Scale the drive values
        scaleDrive(appliedScale/maxDriveValue, driveValues);
        if (debugOn) Log.d(logTag, "Drive vector, no spin after scaling: " + Arrays.toString(driveValues));
        //Now compare the spin signal to the speed setting
        double conditionedSpinSignal = (speed.getTurnSpeed() < spinSig) ? speed.getTurnSpeed() : spinSig;

        //Note, now that spin is field orientated, indices 0&2 are negative while 1&3 are positive
        driveValues[0] -= conditionedSpinSignal;
        driveValues[1] += conditionedSpinSignal;
        driveValues[2] -= conditionedSpinSignal;
        driveValues[3] += conditionedSpinSignal;

        if (debugOn) Log.d(logTag, "Drive vector, with spin no scaling: " + Arrays.toString(driveValues));

        //Now scale the drive signals to makes sure no signal is greater than 1
        maxDriveValue = findMaxAbsValue(driveValues);
        scaleDrive(1/maxDriveValue, driveValues);
        if (debugOn) Log.d(logTag, "Output Drive vector, with spin and scaling: " + Arrays.toString(driveValues));

        //  Finally, apply the calculated drive vectors
        ArrayList<DcMotor> driveMotors = getDriveMotors();
        if (driveMotors.size()>0) {
            int i = 0;
            for (DcMotor m : driveMotors) {
                m.setPower(i++);
            }
        } else {
            if(debugOn) Log.d(logTag, "No drive motors found");
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
        boolean debugOn = true;
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

}
