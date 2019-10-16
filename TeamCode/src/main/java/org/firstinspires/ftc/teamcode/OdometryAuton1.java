package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@Autonomous
public class OdometryAuton1 extends eBotsOpMode2019 {

    @Override
    public void runOpMode(){

        ArrayList<DcMotor> motorList= new ArrayList<>();
        initializeDriveMotors(motorList, false);

        //  Set starting pose and capture gyro offset
        final Pose startingPose = new Pose(Pose.StartingPose.RED_FOUNDATION);

        //  Set target pose
        final Pose targetPose = new Pose(startingPose.getX(), 0.0, startingPose.getHeading());

        final double pGain = 0.3;
        final double iGain = 0.15;

        Integer loopCount = 0;

        Double pSignal;
        Double iSignal;
        Double computedSignal=0.0;
        Double previousSignalSign=0.0;
        Double outputSignal;

        Boolean isSaturated = true;
        Boolean driveSignalSignChange = false;  //  When heading is locked, the headingError only
                                        //  gets recalculated when driveSignal changes sign
                                        //  This allows for the I portion of the PID controller
                                        //  to overshoot the target position


        //This is the angle the robot needs to travel relative to the robot's reference frame
        //This assumes that forward is 0 degress, 90 degrees is traveling left
        double robotTravelAngleRad;


        //These values get calculated by calculateDriveVector function
        double[] driveValues = new double[4];  //Array of values for the motor power levels

        EncoderTracker forwardTracker = new EncoderTracker(motorList.get(0), EncoderTracker.RobotOrientation.FORWARD);

        EncoderTracker lateralTracker = new EncoderTracker(motorList.get(1), EncoderTracker.RobotOrientation.LATERAL);

        //  *********  INITIALIZE FOR FIRST PASS THROUGH LOOP   *****************
        //  Create currentPose, which is the tracked position from start to target
        //  This is a special type of pose that is intended to track the path of travel
        //  It has a targetPose, which is the intended destination
        //  It also has an error object which tracks it's status relative to targetPose
        TrackingPose currentPose = new TrackingPose(startingPose, targetPose);

        //This is called only once to document offset of gyro from field coordinate system
        currentPose.setInitialGyroOffset(getCurrentHeading());

        //Get the initial error value
        //NOT NEEDED, Initial error is calculated when Tracking Pose instantiated
        //currentPose.calculatePoseError();

        //xError = currentPose.getXError(targetPose);
        //yError = currentPose.getYError(targetPose);
        //totalError = Math.hypot(xError, yError);


        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();

        writeOdometryTelemetry(loopCount, currentPose.getSignedError(), currentPose);

        while(opModeIsActive() &&
                (currentPose.getErrorMagnitude() + Math.abs(currentPose.getErrorSum()) > 0.2)){

            if (loopCount>0){  //For every iteration after the first
                EncoderTracker.getNewPose(currentPose);               //update position if not first loop
                currentPose.setHeadingFromGyro(getCurrentHeading());  //Update heading with robot orientation
                currentPose.calculatePoseError();                     //Update error object

                //Compute a new ErrorSum for the Integrator
                //  if BOTH of the following conditions are met, don't add the integrator
                //  -->signal can't be saturated in previous iteration (!isSaturated)
                //  -->error sign same as error Sum (evalutes to true if errorSum 0)
                currentPose.updateErrorSum(isSaturated);
            }

            pSignal = pGain * currentPose.getErrorMagnitude();
            iSignal = iGain * currentPose.getErrorSum();

            previousSignalSign = Math.signum(computedSignal);
            computedSignal = pSignal + iSignal;
            isSaturated = computedSignal > 1 ? true : false;

            if (    loopCount == 0 |
                    (previousSignalSign == Math.signum(computedSignal)))
            {
                driveSignalSignChange = false;
            } else {
                driveSignalSignChange = true;
            }

            currentPose.setHeadingErrorLocked(isSaturated, driveSignalSignChange);
            outputSignal = (isSaturated) ? 1 : computedSignal;

            //Which way the robot needs to travel relative to the robot forward direction
            robotTravelAngleRad = currentPose.getGyroDriveDirectionRad();

            calculateFieldOrientedDriveVector(currentPose.getHeadingErrorRad(), robotTravelAngleRad,outputSignal,0,driveValues);
            //Now actually assign the calculated drive values to the motors in motorList

            for(int i=0;i<motorList.size();i++){
                motorList.get(i).setPower(driveValues[i]);
            }

            loopCount++;

            writeOdometryTelemetry(loopCount, currentPose.getSignedError(), currentPose);
        }


    }

    private void writeOdometryTelemetry(Integer loopCount, double totalError, TrackingPose currentPose) {
        telemetry.addData("Loop Count: ", loopCount);
        telemetry.addData("Total Error: ", currentPose.getSignedError());
        telemetry.addData("Heading Locked: ", currentPose.isHeadingErrorLocked());
        telemetry.addData("Heading Error: ", currentPose.getHeadingError());

        telemetry.update();
    }

}
