package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@TeleOp
@Disabled
public class TeleOp_Competition_FO extends eBotsOpMode2019 {

    /**
     * FIELD ORIENTED VERSION OF COMPETITION_BETA_V2
     */


    @Override
    public void runOpMode(){

        //***************************************************************
        //Initialize the drive motors
        //***************************************************************
        ArrayList<DcMotor> motorList= new ArrayList<>();
        if (motorList.size()>1) motorList.clear();  //Make sure there aren't any items in the list
        initializeDriveMotors(motorList, true);
        initializeImu();
        boolean driveFieldOriented = false;

        //***************************************************************
        //Initialize the variables that are being used in the main loop
        //***************************************************************

        //This first group is for basic navigation
        double driveX;      //Left or right movement
        double driveY;      //Forward or back movement
        double spin;        //Rotation

        //These are calculated based on the stick inputs
        double r;       //length of radius for driveX and driveY
        double robotAngle;      //adjust of angle to account for mecanum drive

        //This are used to refine the input for driver control
        double fineAdjust;                          //Used for super-slow mode
        final double fineAdjustThreshold = 0.3;    //Avoid trivial amounts of speed reduction with threshold value
        final double fineAdjustMaxReduction = 0.65; //Don't allow drive to be fully negated
        boolean fineAdjustOn = false;               //Flag if fine adjust is activated
        boolean speedBoostOn = false;               //Maximize motor drive speeds if pressed
        final double motorThreshold=0.10;
        double spinScaleFactor = 0.4;

        //These values get calculated by calculateDriveVector function
        double[] driveValues = new double[4];  //Array of values for the motor power levels
        double maxValue;                        //Identify ax value in driveValues array

        //***************************************************************
        //Initialize Manipulator Arm variables
        //***************************************************************
        initializeManipMotors();
        //***************************************************************
        //Initialize Limit Switches
        //***************************************************************

        initializeLimitSwitches();

        //***************************************************************
        //Initialize the variables that are being used in the main loop
        //***************************************************************
        StopWatch rakeTimer = new StopWatch();
        StopWatch rotate1ClawTimer = new StopWatch();
        StopWatch rotate2ClawTimer = new StopWatch();
        StopWatch clawTimer = new StopWatch();
        StopWatch lifterTimer = new StopWatch();
        Boolean rakeBusy = false;
        Boolean rotate1ClawBusy = false;
        Boolean rotate2ClawBusy = false;
        Boolean clawBusy = false;
        Boolean lifterBusy = false;
        Boolean holdLifterPosition = false;


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
        findLifterZero();

        while(opModeIsActive()) {
            if (gamepad1.start && gamepad1.left_bumper){
                driveFieldOriented = true;
            } else if (gamepad1.start){
                driveFieldOriented = false;
            }

            if (!driveFieldOriented){
                processDriverControls();
            } else {
                processDriverControlsFieldOriented();
            }

            processManualManipControls();
            processManualManipControls();
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
