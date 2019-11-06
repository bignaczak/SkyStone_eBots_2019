package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Set;

import static java.lang.String.format;

@Autonomous

public class Auton_Blue_Foundation_W1 extends eBotsAuton2019 {

    /****************************************************************
    //******    CONFIGURATION PARAMETERS
    //***************************************************************/
    private Alliance alliance = Alliance.BLUE;
    private FieldSide fieldSide = FieldSide.FOUNDATION;
    private Speed speedConfig = Speed.SLOW;
    private GyroSetting gyroConfig = GyroSetting.EVERY_LOOP;
    private SoftStart softStartConfig = SoftStart.MEDIUM;
    private Accuracy accuracyConfig = Accuracy.STANDARD;


    /****************************************************************/


    @Override
    public void runOpMode(){
        /** APPLY CONFIG     **************/
        setGyroConfiguration(gyroConfig);      //Set gyro parameters and initialize
        setSpeedConfiguration(speedConfig);    //Set speed and PID gain limits
        setSoftStartConfig(softStartConfig);    //Set the soft start settings
        setAccuracyLimits(accuracyConfig);      //Set the accuracy limits

        ArrayList<DcMotor> motorList = new ArrayList<>();   //List of motors

        //  Setup motors & encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        }else{
            initializeDriveMotors(motorList, true);
            initializeEncoderTrackers(motorList);           //actual encoders
            initializeManipMotors();
        }

        Integer wayPoseIndex = 1;
        wayPoses = new ArrayList<>();
        setWayPoses(wayPoses, alliance, fieldSide);
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
            currentPose.setInitialGyroOffset(getCurrentHeading());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }


        String loopMetrics = "Loop Initialized";
        writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);

        /***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //**************************************************************/
        waitForStart();

        //Travel first leg
        Log.d("BTI_runOpMode", "~~~~~~~~~~~~~Starting LegStarting Leg " + wayPoseIndex.toString());

        raiseLifter();
        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************

        TrackingPose endPose = travelToNextPose(currentPose, motorList);
        Double previousInitialGyroOffset = currentPose.getInitialGyroOffset();
        if(!simulateMotors) stopMotors(motorList);
        wayPoseIndex++;

        Double currentPoseHeadingError;
        while(wayPoseIndex< wayPoses.size()) {

            executePostMoveActivity(currentPose);
            correctHeading(currentPose, motorList);
            //Correct heading angle if not good enough

            //Set ending of previous leg as the starting pose for new leg
            Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
            currentPose = new TrackingPose(startPose, wayPoses.get(wayPoseIndex));
            setInitialOffsetForTrackingPose(imu, currentPose, previousInitialGyroOffset);

            Log.d("BTI_runOpMode", "~~~~~~~~~~~~~Starting Leg " + wayPoseIndex.toString());
            endPose = travelToNextPose(currentPose, motorList);
            if(!simulateMotors) stopMotors(motorList);
            wayPoseIndex++;
        }
        correctHeading(currentPose, motorList);

        StopWatch timer = new StopWatch();
        timer.startTimer();
        loopMetrics = "All Points Completed";

        while (opModeIsActive() && timer.getElapsedTimeSeconds() < 5){
            writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);
        }
        //*************************************************8

    }

}
