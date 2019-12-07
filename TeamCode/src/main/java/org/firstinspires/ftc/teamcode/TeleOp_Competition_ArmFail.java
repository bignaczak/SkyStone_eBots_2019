package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@TeleOp
public class TeleOp_Competition_ArmFail extends eBotsOpMode2019 {

    /**
     * Change lifter behavior for going down to RUN_WITHOUT_ENCODERS
     *
     * --> Reversed the rollerGripper directions.  Right_trigger = Out, Right_bumper = IN
     * --> AutoGrab is now left bumper and right bumper
     * --> Dpad_Down sets lifter to grab height
     * --> Make super-slow mo a little faster (0.65 reduction instead of 0.75)
     * --> Rake transferred to CRServo (rake2) and primarily controlled by driver
     * --> yoyo added to control tape measure for parking in end game
     * --> Controller input processing handed to method in abstract model
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
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        writeTelemetry();


        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();
        //  To protect for the case that the arm isn't extended, first raise lifter
        raiseLifterToExtendArm();
        //  Then find the zero point
        findLifterZero();

        while(opModeIsActive()) {
            //GAMEPAD1 INPUTS
            //----------------------------------------
            //Get the drive inputs from the controller
            processDriverControls();

            //GAMEPAD2 INPUTS
            processManualManipControls();
            processAutomatedManipControls();

            writeTelemetry();
        }
    }

    private void writeTelemetry(){
        telemetry.addData("Lifter Target", lifter.getTargetPosition());
        telemetry.addData("Lifter actual position", lifter.getCurrentPosition());
        telemetry.addData("Lifter error", Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()));
        telemetry.addData("Roller Gripper Power", rollerGripper.getPower());
        String switchStatus = lifterLimit1.getState() + " / " + lifterAtBottom.getState();
        telemetry.addData("Switch1 / Switch2", switchStatus);
        telemetry.addData("Lift Power Limit:", lifterPowerLevel);
        telemetry.addData("Lift Target:", lifterPosition);
        telemetry.update();
    }
}
