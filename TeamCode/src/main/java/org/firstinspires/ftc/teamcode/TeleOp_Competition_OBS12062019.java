package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import java.util.ArrayList;

@TeleOp
@Disabled
public class TeleOp_Competition_OBS12062019 extends eBotsOpMode2019 {

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
        StopWatch lifterTimer = new StopWatch();
        Boolean rakeBusy = false;
        Boolean lifterBusy = false;


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
            //GAMEPAD1 INPUTS
            //----------------------------------------
            //Get the drive inputs from the controller
            //  [LEFT STICK]   --> Direction and Speed
            //  [RIGHT STICK]  --> X Direction dictates spin rate to rotate about robot center
            //  [LEFT TRIGGER] --> Variable reduction in robot speed to allow for fine position adjustment
            //  [RIGHT BUMPER] --> Speed boost, maximized motor drive speed

            driveX = gamepad1.left_stick_x;        //Read left stick position for left/right motion
            driveY = -gamepad1.left_stick_y;       //Read left stick position for forward/reverse Motion
            spin = gamepad1.right_stick_x * spinScaleFactor; //This is used to determine how to spin the robot
            fineAdjust = gamepad1.left_trigger;     //Pull to slow motion
            speedBoostOn = gamepad1.right_bumper;   //Push to maximize motor drives

            //r gives the left stick's offset from 0 position by calculating hypotenuse of x and y offset
            r = Math.hypot(driveX, driveY);

            //Robot angle calculates the angle (in radians) and then subtracts pi/4 (45 degrees) from it
            //The 45 degree shift aligns the mecanum vectors for drive
            robotAngle = Math.atan2(driveY, driveX) - Math.PI / 4;
            calculateDriveVector(r, robotAngle, spin, driveValues);     //Calculate motor drive speeds

            //Now allow for fine maneuvering by allowing a slow mode when pushing trigger
            //Trigger is an analog input between 0-1, so it allows for variable adjustment of speed
            //Now scale the drive values based on the level of the trigger
            //We don't want to trigger to allow the joystick to be completely negated
            //And we don't want trivial amounts of speed reduction
            //Initialized variable above Set threshold value to ~0.2 (fineAdjustThreshold)
            // and only allow 80% reduction of speed (fineAdjustMaxReduction)
            if (fineAdjust >= fineAdjustThreshold) {
                fineAdjustOn = true;
                fineAdjust *= fineAdjustMaxReduction;
            } else {
                fineAdjustOn = false;
                fineAdjust = 0;
            }
            fineAdjust = 1 - fineAdjust;

            if (fineAdjustOn) scaleDrive(fineAdjust, driveValues);    //Apply Fine Adjust


            //Now maximize speed by applying a speed boost
            //The drive calculation sometimes doesn't set the peak drive to 1, this corrects that
            if (!fineAdjustOn & speedBoostOn) {      //Fine Adjust mode takes precedent over speed boost
                maxValue = findMaxAbsValue(driveValues);  //See what the max drive value is set to
                if (maxValue < 1 & maxValue > 0)
                    scaleDrive(1 / maxValue, driveValues);  //If max value isn't 1, Scale the values up so max is 1
            }

            //Now actually assign the calculated drive values to the motors in motorList
            int i = 0;
            for (DcMotor m : motorList) {
                m.setPower(driveValues[i]);
                i++;
            }

            //GAMEPAD2 INPUTS
            //----------------------------------------
            //Y - raise foundation rake (has override)
            //A - lower foundation rake (has override)
            //dpad_Down - Set lifter height to grab stone
            //left Stick - lifter up and down


            //----------RAKE INPUTS----------------
            if (!rakeBusy && gamepad2.y && !gamepad2.left_bumper){
                foundationRakePosition = RAKE_UP;
                rakeTimer.startTimer();
                rakeBusy = true;
                foundationRake.setPosition(foundationRakePosition);

            } else if (!rakeBusy && gamepad2.a && !gamepad2.left_bumper){
                foundationRakePosition = RAKE_DOWN;
                rakeTimer.startTimer();
                rakeBusy = true;
                foundationRake.setPosition(foundationRakePosition);

                //these 2 are override conditions
            } else if(!rakeBusy && gamepad2.a && gamepad2.left_bumper) {
                if (foundationRakePosition < 1.0) foundationRakePosition += rakeIncrement;
                rakeTimer.startTimer();
                rakeBusy = true;
                foundationRake.setPosition(foundationRakePosition);
            } else if(!rakeBusy && gamepad2.y && gamepad2.left_bumper) {
                if (foundationRakePosition > 0.0) foundationRakePosition -= rakeIncrement;
                rakeTimer.startTimer();
                rakeBusy = true;
                foundationRake.setPosition(foundationRakePosition);
            } else if (rakeBusy && rakeTimer.getElapsedTimeMillis()>timerLimit){
                rakeBusy = false;
            }

            //----------rollerGripper INPUTS----------------
            if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                //----------Ingest----------------
                rollerGripper.setPower(-rollerGripperPowerLevel);
            } else if (gamepad2.right_trigger > 0.3 && !gamepad2.left_bumper) {
                //----------release----------------
                rollerGripper.setPower(rollerGripperPowerLevel);
            } else if(gamepad2.left_bumper && gamepad2.right_bumper){
                //----------Initiate AutoGrab----------------
                autoGrabStone(motorList);
            } else if(gamepad2.left_bumper && gamepad2.right_trigger > 0.3){
                //----------Initiate AutoRelease----------------
                autoReleaseStone(eBotsAuton2019.Speed.SLOW);
            }
            else rollerGripper.setPower(0.0);




            //----------Lifter INPUTS----------------
            // INTENDED FUNCTION
            //  1) Move down if left stick pushing down and not at limit
            //  2) Move up if left stick pushing up and not at limit
            //  3) Hold position if not moving stick
            lifterUserInput = -gamepad2.left_stick_y;   //change sign for readability

            if (lifterUserInput < -0.3){
                //This is for going down
                //  1) Move down if left stick pushing down and not at limit
                moveLifter(LifterDirection.DOWN);

            } else if (lifterUserInput > 0.3){
                //This is for going up
                //  2) Move up if left stick pushing up and not at limit
                moveLifter(LifterDirection.UP);

            } else if(gamepad2.dpad_down) {
                setLifterHeightToGrabStone();
            }else if (Math.abs(lifterUserInput) <= 0.3) {
                //  3) Hold position if not moving stick
                holdLifterAtCurrentPosition();
            }

            //----------Adjust lifter speed----------------
            if (!lifterBusy && gamepad2.left_bumper && gamepad2.right_stick_y < -0.3){
                lifterPowerLevel += 0.05;
                if(lifterPowerLevel>1.0) lifterPowerLevel = 1.0;
                lifter.setPower(lifterPowerLevel);
                lifterBusy = true;
                lifterTimer.startTimer();
            } else if (!lifterBusy && gamepad2.left_bumper && gamepad2.right_stick_y > 0.3) {
                lifterPowerLevel -= 0.05;
                if(lifterPowerLevel<0.0) lifterPowerLevel = 0.0;
                lifter.setPower(lifterPowerLevel);
                lifterBusy = true;
                lifterTimer.startTimer();
            } else if (lifterBusy && lifterTimer.getElapsedTimeMillis()>lifterTimerLimit){
                lifterBusy = false;
            }
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
