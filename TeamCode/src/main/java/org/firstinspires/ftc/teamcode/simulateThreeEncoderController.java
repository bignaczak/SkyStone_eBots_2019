package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.eBotsMotionController.moveToTargetPose;

@Autonomous
public class simulateThreeEncoderController extends eBotsAuton2019 {

    /****************************************************************
     //******    CONFIGURATION PARAMETERS
     //***************************************************************/
    private Alliance alliance = Alliance.BLUE;
    private FieldSide fieldSide = FieldSide.FOUNDATION_V2;
    private Speed speedConfig = Speed.MEDIUM;
    private GyroSetting gyroConfig = GyroSetting.NONE;
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
        simulateMotors = true;

        String logTag = "BTI_sim3OpMode";
        Log.d(logTag, "Starting OpMode...");
        //  Setup motors & encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        }
        Log.d(logTag, "Encoders initialized " + EncoderTracker.getEncoderTrackerCount() + " found");

        Pose startPose = new Pose(0.0, 0.0, 0.0);
        Pose targetPose = new Pose(40.0, 30.0, 90.0);
        TrackingPose trackingPose = new TrackingPose(startPose, targetPose);

        if (useGyroForNavigation) {
            trackingPose.setInitialGyroOffset(getGyroReadingDegrees());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            trackingPose.setInitialGyroOffset(0.0);
        }

        telemetry.clear();
        telemetry.addLine("Ready...");
        telemetry.update();
        waitForStart();

        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig);

    }
}
