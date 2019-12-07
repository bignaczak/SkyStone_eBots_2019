package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@Autonomous


public class Blue_Foundation_V2 extends eBotsAuton2019 {

    @Override
    public void runOpMode(){
        // Set config variables
        alliance = Alliance.BLUE;
        fieldSide = FieldSide.FOUNDATION_V2;
        speedConfig = Speed.MEDIUM;
        gyroConfig = GyroSetting.EVERY_LOOP;
        softStartConfig = SoftStart.MEDIUM;
        accuracyConfig = Accuracy.STANDARD;
        delayedStart = DelayedStart.NO;


        /** APPLY CONFIG     **************/
        setGyroConfiguration(gyroConfig);      //Set gyro parameters and initialize
        setSpeedConfiguration(speedConfig);    //Set speed and PID gain limits
        setSoftStartConfig(softStartConfig);    //Set the soft start settings
        setAccuracyLimits(accuracyConfig);      //Set the accuracy limits

        ArrayList<DcMotor> motorList = new ArrayList<>();   //List of motors

        String logTag = "BTI_FOUNDATION";
        Log.d(logTag, "Starting OpMode...");
        //  Setup motors & encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        }else{
            initializeDriveMotors(motorList, true);
            initializeEncoderTrackers(motorList);           //actual encoders
            initializeManipMotors();
            initializeLimitSwitches();          //limit switches
            initializeDistanceSensors();        //distance sensors

        }

        Integer wayPoseIndex = 1;
        wayPoses = new ArrayList<>();
        setWayPoses(wayPoses, alliance, fieldSide, DelayedStart.NO);
        //  *********  INITIALIZE FOR FIRST PASS THROUGH LOOP   *****************
        //  Create currentPose, which is the tracked position from start to target
        //  This is a special type of pose that is intended to track the path of travel
        //  It has a targetPose, which is the intended destination
        //  It also has an error object which tracks it's status relative to targetPose

        TrackingPose currentPose;
        if (wayPoses.size() > 1) {  //Make sure there are at least 2 points
            currentPose = new TrackingPose(wayPoses.get(0), wayPoses.get(wayPoseIndex));
        } else {    //Set end point same as start point (road to nowhere)
            currentPose = new TrackingPose(wayPoses.get(0), wayPoses.get(0));
        }

        //  Instantiate gyro object if being utilized
        //This is called only once to document offset of gyro from field coordinate system

        if (useGyroForNavigation) {
            currentPose.setInitialGyroOffset(getGyroReadingDegrees());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }
        Double initialGyroOffset = currentPose.getInitialGyroOffset();

        Log.d(logTag, "First Waypose set");

        //***************************************************************
        //  Open up the webcam
        //***************************************************************
        prepWebcam();


        String loopMetrics = "Loop Initialized";
        writeOdometryTelemetry(loopMetrics, currentPose);


        /***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //**************************************************************/
        waitForStart();

        //Travel first leg
        Log.d(logTag, "~~~~~~~~~~~~~Starting LegStarting Leg " + wayPoseIndex.toString());

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************

        TrackingPose endPose = travelToNextPose(currentPose);
        wayPoseIndex++;

        Double currentPoseHeadingError;
        while(opModeIsActive() && wayPoseIndex< wayPoses.size()) {

            //Correct heading angle if not within tolerance limits set in accuracyConfig
            correctHeading(currentPose);
            //Perform actions specified in the targetPose of the path leg that was just completed
            executePostMoveActivity(currentPose, alliance);

            //Set ending of previous leg as the starting pose for new leg
            //  Position and pose are taken from the TrackingPose object returned by travelToNextPose
            Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
            currentPose = new TrackingPose(startPose, wayPoses.get(wayPoseIndex));

            //Must set the initialGryOffset
            //This routine first sets the heading and then sets the initialGyroOffset
            //This is likely redundant and can be replaced with just a call to setInitialGyroOffset
            //Todo: Verify that copying the initialGyroOffset works and delete setInitialOffset...
            currentPose.copyInitialGyroOffsetBetweenLegs(initialGyroOffset);
            //setInitialOffsetForTrackingPose(imu, currentPose, initialGyroOffset);

            Log.d(logTag, "~~~~~~~~~~~~~Starting Leg " + wayPoseIndex.toString() + " to " +
                    currentPose.getTargetPose().toString());
            Log.d(logTag, "Start Position " + currentPose.toString());
            endPose = travelToNextPose(currentPose);
            Log.d(logTag, "Leg completed, new position " + endPose.toString());
            wayPoseIndex++;
        }
        Log.d(logTag, "All Legs Completed, current Position " + endPose.toString());
        correctHeading(currentPose);

        //  Verify all drive motors are stopped
        if(!simulateMotors) stopMotors();

        //  Close vuforia webcam interface
        if (tfod != null) {
            tfod.shutdown();
        }


    }

}
