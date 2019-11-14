package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@Autonomous

public class Auton_Detect extends eBotsAuton2019 {

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

        //This is called only once to document offset of gyro from field coordinate system

        if (useGyroForNavigation) {
            currentPose.setInitialGyroOffset(getGyroReadingDegrees());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }
        Double initialGyroOffset = currentPose.getInitialGyroOffset();


        String loopMetrics = "Loop Initialized";
        writeOdometryTelemetry(loopMetrics, currentPose);

        /***************************************************************
         * Prepare the webcam
         ***************************************************************/

        prepWebcam();

        /***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //**************************************************************/
        waitForStart();

        while (opModeIsActive()){
            scanForSkystone();
        }

        //  Close vuforia webcam interface
        if (tfod != null) {
            tfod.shutdown();
        }


    }

}
