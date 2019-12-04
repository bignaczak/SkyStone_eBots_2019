package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

import static java.lang.String.format;

@TeleOp
public class TeleOp_EncoderCalibration extends eBotsOpMode2019 {

    /**
     * Change lifter behavior for going down to RUN_WITHOUT_ENCODERS
     *
     * --> Reversed the rollerGripper directions.  Right_trigger = Out, Right_bumper = IN
     * --> AutoGrab is now left bumper and right bumper
     * --> Dpad_Down sets lifter to grab height
     * --> Make super-slow mo a little faster (0.65 reduction instead of 0.75)
     */


    @Override
    public void runOpMode(){

        //***************************************************************
        //Initialize the drive motors
        //***************************************************************
        ArrayList<DcMotor> motorList= new ArrayList<>();
        if (motorList.size()>1) motorList.clear();  //Make sure there aren't any items in the list
        initializeDriveMotors(motorList, true);

        //***************************************************************
        //Initialize Manipulator Arm variables
        //***************************************************************
        initializeManipMotors();

        //***************************************************************
        //Initialize Limit Switches
        //***************************************************************
        initializeLimitSwitches();

        //***************************************************************
        //Initialize Encoders
        //***************************************************************
        initializeEncoderTrackers(getDriveMotors());


        //***************************************************************
        //Initialize Imu
        //***************************************************************
        initializeImu();


        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        writeTelemetry();


        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();
        //  Then find the zero point
        //findLifterZero();

        while(opModeIsActive()) {
            //GAMEPAD1 INPUTS
            //----------------------------------------
            //Get the drive inputs from the controller
            processDriverControls();

            //GAMEPAD2 INPUTS
            processManualManipControls();

            writeTelemetry();
        }
    }



    private void writeTelemetry(){
        telemetry.addData("forwardTracker: ", forwardTracker.toString());
        telemetry.addData("forwardTracker2: ", forwardTracker2.toString());
        telemetry.addData("lateralTracker: ", lateralTracker.toString());
        if (imu != null) telemetry.addData("imu: ", format("%.2f", getGyroReadingDegrees()));

        String switchStatus = lifterLimit1.getState() + " / " + lifterAtBottom.getState();
        telemetry.addData("Switch1 / Switch2", switchStatus);
        telemetry.addData("Rake Position: ", foundationRake.getPosition());
        telemetry.update();
    }

    private void writeAutoGrabTelemetry(String status){
        telemetry.addData("Status", status);
        telemetry.addData("Lifter Target", lifter.getTargetPosition());
        telemetry.addData("Lifter actual position", lifter.getCurrentPosition());
        telemetry.addData("Lifter error", Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()));
        telemetry.addData("Roller Gripper Power", rollerGripper.getPower());
        String switchStatus = lifterLimit1.getState() + " / " + lifterAtBottom.getState();
        telemetry.addData("Switch1 / Switch2", switchStatus);
        telemetry.update();

    }

}
