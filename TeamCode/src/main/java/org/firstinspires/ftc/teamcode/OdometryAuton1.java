package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@Autonomous
public class OdometryAuton1 extends eBotsOpMode2019 {

    private static Double initialGyroOffset;

    public Double getInitialGyroOffset(){
        return initialGyroOffset;
    }

    @Override
    public void runOpMode(){

        ArrayList<DcMotor> motorList= new ArrayList<>();
        initializeDriveMotors(motorList, false);

        final Pose startingPose = new Pose(Pose.StartingPose.RED_FOUNDATION);
        final Pose targetPose = new Pose(startingPose.getX(), 0.0, startingPose.getHeading());
        Pose currentPose = new Pose(startingPose.getX(), startingPose.getY(), startingPose.getHeading());
        initialGyroOffset = getCurrentHeading() + startingPose.getHeading();

        final double pGain = 0.3;
        final double iGain = 0.15;

        Integer loopCount = 0;

        double xError;
        double yError;
        double totalError;
        double headingError;  //In Degrees
        double errorSum = 0;

        double pSignal;
        double iSignal;
        double computedSignal;
        double outputSignal;

        boolean isSaturated;
        boolean hasErrorSumChangedSign;


        //This is the angle the robot needs to travel relative to the robot's angle
        //This assumes that forward is 0 degress, 90 degrees is traveling left
        double robotTravelAngle;

        //this is the angle that must be passed into the calculateDrive
        double robotAngle;

        //These values get calculated by calculateDriveVector function
        double[] driveValues = new double[4];  //Array of values for the motor power levels

        EncoderTracker forwardTracker = new EncoderTracker(motorList.get(0), EncoderTracker.RobotOrientation.FORWARD);

        EncoderTracker lateralTracker = new EncoderTracker(motorList.get(1), EncoderTracker.RobotOrientation.LATERAL);
        //Get the initial error value
        xError = currentPose.getXError(targetPose);
        yError = currentPose.getYError(targetPose);
        totalError = Math.hypot(xError, yError);


        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();

        writeOdometryTelemetry(loopCount, totalError);

        while(opModeIsActive() &&
                Math.abs(totalError + errorSum) <0.1){


            if (loopCount>0){
                //update position if not first loop
                currentPose = EncoderTracker.getNewPose(currentPose);
                xError = currentPose.getXError(targetPose);
                yError = currentPose.getYError(targetPose);
                totalError = Math.hypot(xError, yError);
            }

            pSignal = pGain * totalError;
            iSignal = iGain * errorSum;
            computedSignal = pSignal + iSignal;
            isSaturated = (computedSignal > 1) ? true : false;

            if (isSaturated){
                outputSignal = 1;
            } else {
                outputSignal = computedSignal;
            }

            //Which way the robot needs to travel relative to the field
            headingError = Math.toDegrees(Math.atan2(yError, xError));

            //Which way the robot needs to travel relative to the robot forward direction
            robotTravelAngle = getRobotLocalHeading(headingError, startingPose);

            //Robot angle calculates the angle (in radians) and then subtracts pi/4 (45 degrees) from it
            //The 45 degree shift aligns the mecanum vectors for drive
            robotAngle = Math.toRadians(robotTravelAngle - 45);
            calculateDriveVector(outputSignal, robotAngle, 0, driveValues);     //Calculate motor drive speeds

            //Now actually assign the calculated drive values to the motors in motorList
            int i = 0;
            for (DcMotor m : motorList) {
                m.setPower(driveValues[i]);
                i++;
            }


            loopCount++;

            writeOdometryTelemetry(loopCount, totalError);

        }


    }

    private void writeOdometryTelemetry(Integer loopCount, double totalError) {
        telemetry.addData("Loop Count: ", loopCount);
        telemetry.addData("Total Error: ", totalError);
        telemetry.update();
    }

    public double getRobotFieldOrientedHeading(){
        return getCurrentHeading() + initialGyroOffset;
    }

    private static double getRobotLocalHeading (Double targetFieldHeading, Pose startingPose){
        return targetFieldHeading - startingPose.getHeading();
    }
}
