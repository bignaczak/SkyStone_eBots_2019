package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SampleTeleOpMode_Manip extends eBotsOpMode2019 {

    //------CONSTANTS FOR SERVO POSITIONS
    private final Double RAKE_DOWN = 0.60;
    private final Double RAKE_UP = 0.25;

    private final Double CLAW_OPEN = 0.390;
    private final Double CLAW_CLOSED = 0.18;

    private final Double ROTATE1_CLAW_FORWARD = 0.846;
    private final Double ROTATE1_CLAW_90 = 0.319;
    private final Double ROTATE_CLAW_PACK = 0.015;


    private final Double ROTATE2_CLAW_FORWARD = 0.798;
    private final Double ROTATE2_CLAW_90RIGHT = 0.289;


    //---------------------------------------------

    private double foundationRakePosition;
    private double rotate1ClawPosition;
    private double rotate2ClawPosition;
    private double clawPosition;

    private double timerLimit = 100;  //ms
    private double clawTimerLimit = 250;  //ms

    private double rakeIncrement = 0.015;
    private double clawIncrement = 0.01;
    private double rotate1ClawIncrement = 0.015;
    private double rotate2ClawIncrement = 0.015;

    private Boolean clawOpen = true;





    @Override
    public void runOpMode(){

    Servo foundationRake;
    Servo rotate1ClawServo;
    Servo rotate2ClawServo;
    Servo claw;

    foundationRake = hardwareMap.get(Servo.class, "foundationRake");
    rotate1ClawServo = hardwareMap.get(Servo.class, "rotate1Claw");
    rotate2ClawServo = hardwareMap.get(Servo.class, "rotate2Claw");
    claw = hardwareMap.get(Servo.class, "claw");

    foundationRakePosition = foundationRake.getPosition();
    rotate1ClawPosition = rotate1ClawServo.getPosition();
    rotate2ClawPosition = rotate2ClawServo.getPosition();
    clawPosition = claw.getPosition();

    //***************************************************************
    //Initialize the variables that are being used in the main loop
    //***************************************************************
    StopWatch rakeTimer = new StopWatch();
    StopWatch rotate1ClawTimer = new StopWatch();
    StopWatch rotate2ClawTimer = new StopWatch();
    StopWatch clawTimer = new StopWatch();
    Boolean rakeBusy = false;
    Boolean rotate1ClawBusy = false;
    Boolean rotate2ClawBusy = false;
    Boolean clawBusy = false;


    //***************************************************************
    //  END OF OPMODE INITIALIZATION
    //  Wait for the game to start(driver presses PLAY)
    //***************************************************************
    writeTelemetry(foundationRake,  rotate1ClawServo, rotate2ClawServo, claw);

    waitForStart();

    while(opModeIsActive()) {
        //GAMEPAD2 INPUTS
        //----------------------------------------
        //Y - raise foundation rake
        //A - lower foundation rake
        //X - claw close
        //B - claw open

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
        if (!clawBusy && gamepad2.right_bumper) {
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

        writeTelemetry(foundationRake,  rotate1ClawServo, rotate2ClawServo, claw);

    }


}

    private void writeTelemetry(Servo foundationRake, Servo rotate1ClawServo, Servo rotate2ClawServo, Servo claw){
        telemetry.addData("Rake Position: ", foundationRake.getPosition());
        telemetry.addData("Rotate1 Claw Position:", rotate1ClawServo.getPosition());
        telemetry.addData("Rotate2 Claw Position:", rotate2ClawServo.getPosition());
        telemetry.addData("Claw Position:", claw.getPosition());
        telemetry.update();
    }

}
