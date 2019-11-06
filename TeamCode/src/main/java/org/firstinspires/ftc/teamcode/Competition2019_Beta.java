package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@TeleOp
public class Competition2019_Beta extends eBotsOpMode2019 {

    /** Change the lifter power behavior offset from current position
     *    It gets rid of the timer and only applies increment to current position
     *    Also, it locks current position when releasing stick
     *    Speed of lifter can be adjusted with leftBumper && right_stick_y
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
    //Lifter is initialized in eBots abstract OpMode
    initializeManipMotors();




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


    //***************************************************************
    //  END OF OPMODE INITIALIZATION
    //  Wait for the game to start(driver presses PLAY)
    //***************************************************************
    writeTelemetry(foundationRake,  rotate1ClawServo, rotate2ClawServo, claw);


        //***************************************************************
    //  END OF OPMODE INITIALIZATION
    //  Wait for the game to start(driver presses PLAY)
    //***************************************************************
    waitForStart();
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

        //----------CLAW INPUTS----------------
        if (!clawBusy && gamepad2.right_bumper && !gamepad2.left_bumper) {
            if (clawOpen) {
                clawPosition = CLAW_CLOSED;     //toggle position
            } else {
                clawPosition = CLAW_OPEN;
            }
            clawOpen = !clawOpen;  //toggle the state
            clawTimer.startTimer();
            clawBusy = true;
            claw.setPosition(clawPosition);
        } else if (!clawBusy && gamepad2.right_trigger>0.3 && gamepad2.left_bumper) {
            //if (clawPosition > CLAW_CLOSED) clawPosition -= clawIncrement;
            if (clawPosition > 0.0) clawPosition -= clawIncrement;
            clawTimer.startTimer();
            clawBusy = true;
            claw.setPosition(clawPosition);
        } else if (!clawBusy && gamepad2.right_bumper && gamepad2.left_bumper){
            //if (clawPosition < CLAW_OPEN) clawPosition += clawIncrement;
            if (clawPosition < 1.0) clawPosition += clawIncrement;
            clawTimer.startTimer();
            clawBusy = true;
            claw.setPosition(clawPosition);
        }else if (clawBusy && clawTimer.getElapsedTimeMillis()>clawTimerLimit){
            clawBusy = false;
            //claw.setPosition(0.5);
        }

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

        if (lifterUserInput <0.3){
            //This is for going down
            lifterPosition = lifter.getCurrentPosition() + lifterIncrement;
            if (lifterPosition>0) lifterPosition = 0;
        } else if (lifterUserInput > -0.3){
            //This is for going up
            lifterPosition = lifter.getCurrentPosition() - lifterIncrement;
            if (lifterPosition < lifterMaxPosition) lifterPosition = lifterMaxPosition;
        } else if (Math.abs(lifterUserInput) <= 0.3) {
            lifterPosition = lifter.getCurrentPosition();
        }
        lifter.setTargetPosition(lifterPosition);

        //----------Adjust lifter speed----------------
        if (!lifterBusy && gamepad2.left_bumper && gamepad2.right_stick_y < 0.3){
            lifterPowerLevel += 0.05;
            if(lifterPowerLevel>1.0) lifterPowerLevel = 1.0;
            lifter.setPower(lifterPowerLevel);
            lifterBusy = true;
            lifterTimer.startTimer();
        } else if (!lifterBusy && gamepad2.left_bumper && gamepad2.right_stick_y>0.3) {
            lifterPowerLevel -= 0.05;
            if(lifterPowerLevel<0.0) lifterPowerLevel = 0.0;
            lifter.setPower(lifterPowerLevel);
            lifterBusy = true;
            lifterTimer.startTimer();
        } else if (lifterBusy && lifterTimer.getElapsedTimeMillis()>lifterTimerLimit){
            lifterBusy = false;
        }


        writeTelemetry(foundationRake,  rotate1ClawServo, rotate2ClawServo, claw);

    }


}

    private void writeTelemetry(Servo foundationRake, Servo rotate1ClawServo, Servo rotate2ClawServo, Servo claw){
        telemetry.addData("Lift Power Limit:", lifterPowerLevel);
        telemetry.addData("Lift Target:", lifterPosition);
        telemetry.addData("Lift Motor Position:", lifter.getCurrentPosition());
        telemetry.addData("Lift Motor Target:", lifter.getTargetPosition());
        telemetry.addData("Rake Position: ", foundationRake.getPosition());
        telemetry.addData("Rotate1 Claw Position:", rotate1ClawServo.getPosition());
        telemetry.addData("Rotate2 Claw Position:", rotate2ClawServo.getPosition());
        telemetry.addData("Claw Position:", claw.getPosition());
        telemetry.update();
    }

}
