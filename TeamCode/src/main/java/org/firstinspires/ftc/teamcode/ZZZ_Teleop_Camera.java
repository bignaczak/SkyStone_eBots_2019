package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@TeleOp
@Deprecated
public class ZZZ_Teleop_Camera extends eBotsAuton2019 {


    /****************************************************************
     //******    CONFIGURATION PARAMETERS
     //***************************************************************/
    private GyroSetting gyroConfig = GyroSetting.EVERY_LOOP;

    /****************************************************************/

    /** Change the lifter power behavior offset from current position
     *    It gets rid of the timer and only applies increment to current position
     *    Also, it locks current position when releasing stick
     *    Speed of lifter can be adjusted with leftBumper && right_stick_y
     */

    private Integer blockHeightClicks = -475;

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
        final double fineAdjustMaxReduction = 0.75; //Don't allow drive to be fully negated
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
        //  Open up the webcam
        //***************************************************************
        prepWebcam();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();
        raiseLifterToExtendArm();
        findLifterZero();
        while(opModeIsActive()) {
            scanForSkystone();

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
            //X - rotate2Claw clockwise (2nd Joint) (has override)
            //B - rotate2Claw counterClockwise (2nd Jont) (has override)
            //dpad_Right - rotateClaw clockwise (1st Joint) (has override)
            //dpad_Left - rotateClaw counterclockwise(1st Joint) (has override)
            //dpad_Up - extend Arm
            //dpad_Down - retract Arm
            //right Bumper - toggle claw position (and close on override)
            //right Trigger - (open claw on override)
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


            //----------ROTATE JOINT 1 INPUTS----------------
            if (!rotate1ClawBusy && gamepad2.dpad_right && !gamepad2.left_bumper) {
                rotate1ClawPosition = ROTATE1_CLAW_FORWARD;
                rotate1ClawTimer.startTimer();
                rotate1ClawBusy = true;
                rotate1ClawServo.setPosition(rotate1ClawPosition);
            } else if(!rotate1ClawBusy && gamepad2.dpad_left && !gamepad2.left_bumper){
                rotate1ClawPosition = ROTATE1_CLAW_90;
                rotate1ClawTimer.startTimer();
                rotate1ClawBusy = true;
                rotate1ClawServo.setPosition(rotate1ClawPosition);

                //these 2 are override conditions
            } else if(!rotate1ClawBusy && gamepad2.dpad_right && gamepad2.left_bumper){
                if (rotate1ClawPosition < 1.0) rotate1ClawPosition += rotate1ClawIncrement;
                rotate1ClawTimer.startTimer();
                rotate1ClawBusy = true;
                rotate1ClawServo.setPosition(rotate1ClawPosition);
            } else if(!rotate1ClawBusy && gamepad2.dpad_left && gamepad2.left_bumper){
                if (rotate1ClawPosition > 0.0) rotate1ClawPosition -= rotate1ClawIncrement;
                rotate1ClawTimer.startTimer();
                rotate1ClawBusy = true;
                rotate1ClawServo.setPosition(rotate1ClawPosition);
            } else if (rotate1ClawBusy && rotate1ClawTimer.getElapsedTimeMillis()>timerLimit){
                rotate1ClawBusy = false;
            }

            //----------ROTATE JOINT 2 INPUTS----------------
            if (!rotate2ClawBusy && gamepad2.x && !gamepad2.left_bumper) {
                rotate2ClawPosition = ROTATE2_CLAW_FORWARD;
                rotate2ClawTimer.startTimer();
                rotate2ClawBusy = true;
                rotate2ClawServo.setPosition(rotate2ClawPosition);
            } else if(!rotate2ClawBusy && gamepad2.b && !gamepad2.left_bumper){
                rotate2ClawPosition = ROTATE2_CLAW_90RIGHT;
                rotate2ClawTimer.startTimer();
                rotate2ClawBusy = true;
                rotate2ClawServo.setPosition(rotate2ClawPosition);


            } else if(!rotate2ClawBusy && gamepad2.x && gamepad2.left_bumper){
                if (rotate2ClawPosition < 1.0) rotate2ClawPosition += rotate2ClawIncrement;
                rotate2ClawTimer.startTimer();
                rotate2ClawBusy = true;
                rotate2ClawServo.setPosition(rotate2ClawPosition);
            } else if(!rotate2ClawBusy && gamepad2.b && gamepad2.left_bumper){
                if (rotate2ClawPosition>0.0) rotate2ClawPosition -= rotate2ClawIncrement;
                rotate2ClawTimer.startTimer();
                rotate2ClawBusy = true;
                rotate2ClawServo.setPosition(rotate2ClawPosition);
            } else if (rotate2ClawBusy && rotate2ClawTimer.getElapsedTimeMillis()>timerLimit){
                rotate2ClawBusy = false;
            }

            //----------rollerGripper INPUTS----------------
            if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                rollerGripper.setPower(0.5);
            } else if (gamepad2.right_trigger > 0.3 && !gamepad2.left_bumper) {
                rollerGripper.setPower(-0.5);
            } else rollerGripper.setPower(0.0);


            //----------EXTEND ARM INPUTS----------------
            if (gamepad2.dpad_up) {
                extendArm.setPower(0.3);
            } else if(gamepad2.dpad_down) {
                extendArm.setPower(-0.3);
            } else {
                extendArm.setPower(0.0);
            }

            //----------Lifter INPUTS----------------

            lifterUserInput = -gamepad2.left_stick_y;

            if (lifterUserInput < -0.3){
                //This is for going down
                lifterPosition = lifter.getCurrentPosition() + lifterIncrement;
                if (lifterPosition>0) lifterPosition = 0;
                holdLifterPosition = false;
                lifter.setTargetPosition(lifterPosition);
                lifter.setPower(lifterPowerLevel/2);

            } else if (lifterUserInput > 0.3){
                //This is for going up
                lifterPosition = lifter.getCurrentPosition() - lifterIncrement;
                if (lifterPosition < LIFTER_UPPER_LIMIT) lifterPosition = LIFTER_UPPER_LIMIT;
                holdLifterPosition = false;
                lifter.setTargetPosition(lifterPosition);
                lifter.setPower(lifterPowerLevel);

            } else if (Math.abs(lifterUserInput) <= 0.3) {
                if (!holdLifterPosition) {
                    lifterPosition = lifter.getCurrentPosition();
                    holdLifterPosition = true;
                    lifter.setTargetPosition(lifterPosition);
                }
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

            //----------Initiate AutoGrab----------------
            if (gamepad2.left_bumper && gamepad2.left_trigger > 0.3){
                autoGrabStone(motorList);
            }

            if (gamepad2.left_bumper && gamepad2.right_trigger > 0.3){
                autoReleaseStone(Speed.SLOW);
            }

            //writeTelemetry();

        }

        //  Close vuforia webcam interface
        if (tfod != null) {
            tfod.shutdown();
        }



    }
    @Override
    protected void autoGrabStone(ArrayList<DcMotor> motorList){
        /**
         * Automated routine to grab a block
         * 1) raise the lifter
         * 2) move forward
         * 3) lower lifter while moving grabber wheel
         * 4) stop wheel and lift for travel
         */
        Integer lifterHeightApproach = blockHeightClicks;
        Integer lifterHeightGrab = 0;
        Integer lifterHeightDrive = -150;
        Integer driveTime = 350;
        Double driveSpeed = 0.35;
        final Double rollerGripperSpeed = -0.7;
        Double rollerGripperPulseSpeed;
        Long pulseTimeOut = 750L;
        Long timeout = 2000L;
        StopWatch timer = new StopWatch();
        Integer currentLifterPositionError;


        //  1) raise the lifter
        lifter.setTargetPosition(lifterHeightApproach);
        lifter.setPower(lifterPowerLevel);
        timer.startTimer();
        currentLifterPositionError = lifterHeightApproach - lifter.getCurrentPosition();
        //Note, using lifter.isBusy() for exit loop command took too long, would need to adjust PID gains
        while (opModeIsActive() && Math.abs(currentLifterPositionError)>100
                && timer.getElapsedTimeMillis() < timeout){
            writeAutoGrabTelemetry("Moving lifter to approach position");
            currentLifterPositionError = lifterHeightApproach - lifter.getCurrentPosition();

        }

        //  2) move forward
        //  Start roller while moving forward
        rollerGripper.setPower(rollerGripperSpeed/2);       //half speed

        performDriveStep(0.0,driveSpeed,0.0,driveTime,motorList);


        //  3) lower lifter while moving grabber wheel
            //  don't use run to position mode for this operation
        //lifter.setPower(0.8);  //Note: not using half speed for this maneuever
        //lifter.setTargetPosition(lifterHeightGrab);
        //currentLifterPositionError = lifterHeightGrab - lifter.getCurrentPosition();

        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setPower(0.3);   //
        timer.startTimer();
        Boolean gripperPowerOn = true;

        while(opModeIsActive() && !lifterAtBottom.getState()
                && timer.getElapsedTimeMillis()<timeout){

            rollerGripper.setPower(rollerGripperSpeed/2);       //half speed
            writeAutoGrabTelemetry("Lowering to grab block");
        }
        lifter.setPower(0.0);
        lifterPosition = lifter.getCurrentPosition();
        lifter.setTargetPosition(lifterPosition);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(lifterPowerLevel);

        //pulse the rollerGripper
        timer.startTimer();
        while (opModeIsActive() && timer.getElapsedTimeMillis() < pulseTimeOut){
            //roll the gripper for a while
            rollerGripper.setPower(rollerGripperSpeed);
        }

        //  4) stop wheel and lift for travel
        rollerGripper.setPower(0.0);
        lifter.setTargetPosition(lifterHeightDrive);
        timer.startTimer();

        /*
        while(opModeIsActive() && lifter.isBusy() && timer.getElapsedTimeMillis() < timeout){
            writeAutoGrabTelemetry("Preparing to drive");
        }
         */


    }

    private void pulseRollerGripper(Long pulseTimeOut, Double rollerGripperPulseSpeed){
        StopWatch pulseTimer = new StopWatch();
        pulseTimer.startTimer();
        while (opModeIsActive() && pulseTimer.getElapsedTimeMillis() < pulseTimeOut){
            rollerGripper.setPower(rollerGripperPulseSpeed);
        }

    }

    private void pulseRollerGripper(Integer numPulses, Double rollerGripperPulseSpeed){
        Long timeout = 2000L;
        StopWatch pulseTimer = new StopWatch();
        pulseTimer.startTimer();
        Integer currentLifterPosition = lifter.getCurrentPosition();
        while (opModeIsActive() && currentLifterPosition < 0
                && pulseTimer.getElapsedTimeMillis() < timeout){
            lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rollerGripper.setPower(0.5);
            lifter.setPower(0.5);
            rollerGripper.setPower(rollerGripperPulseSpeed);
            currentLifterPosition = lifter.getCurrentPosition();

        }
        lifterPosition = lifter.getCurrentPosition();
        lifter.setTargetPosition(lifterPosition);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setPower(lifterPowerLevel);

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
