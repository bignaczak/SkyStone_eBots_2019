package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class encoderTrial extends eBotsOpMode2019 {

    @Override
    public void runOpMode(){
        DcMotor motor1;
        Integer encoderCount;
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Encoder Count: ");
        telemetry.update();
        waitForStart();
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()){
            encoderCount = motor1.getCurrentPosition();
            telemetry.addData("Encoder Count: ", encoderCount.toString());
            telemetry.update();
        }


    }
}
