package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.ListIterator;

import static java.lang.String.format;

@Autonomous
public class ZZ_Auton_Encoder_Calibration extends eBotsAuton2019 {


    /****************************************************************
     //******    CONFIGURATION PARAMETERS
     //***************************************************************/
    private Alliance alliance = Alliance.BLUE;
    private FieldSide fieldSide = FieldSide.QUARRY;
    private Speed speedConfig = Speed.MEDIUM;
    private GyroSetting gyroConfig = GyroSetting.EVERY_LOOP;
    private SoftStart softStartConfig = SoftStart.MEDIUM;
    private Accuracy accuracyConfig = Accuracy.STANDARD;


    @Override
    public void runOpMode(){
        /** APPLY CONFIG     **************/
        setGyroConfiguration(gyroConfig);      //Set gyro parameters and initialize
        setSpeedConfiguration(speedConfig);    //Set speed and PID gain limits
        setSoftStartConfig(softStartConfig);    //Set the soft start settings
        setAccuracyLimits(accuracyConfig);      //Set the accuracy limits
        setAllianceObjects(alliance);           //Create objects for Quarry Stones and Foundation
        simulateMotors = false;
        String logTag = "BTI_AutonCameraBeta";

        //***************************************************************
        //Initialize the drive motors
        //***************************************************************
        ArrayList<DcMotor> motorList= new ArrayList<>();
        //  Setup motors & encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        }else{
            initializeDriveMotors(motorList, true);
            initializeEncoderTrackers(motorList);           //actual encoders
            initializeManipMotors();
            initializeLimitSwitches();
        }

        //***************************************************************
        //  Open up the webcam
        //***************************************************************
        prepWebcam();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        String loopMetrics = "Loop Initialized";
        writeOdometryTelemetry();

        waitForStart();

        StopWatch overallTime = new StopWatch();

        while(opModeIsActive()) {
            writeOdometryTelemetry();

        }
    }

    protected void writeOdometryTelemetry() {

        telemetry.addData("forwardTracker: ", forwardTracker.toString());
        telemetry.addData("lateralTracker: ", lateralTracker.toString());
        telemetry.update();
    }


}
