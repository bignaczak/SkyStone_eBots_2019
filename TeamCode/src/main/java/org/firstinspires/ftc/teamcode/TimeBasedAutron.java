package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@Autonomous
public class TimeBasedAutron extends eBotsOpMode2019 {
    private boolean skipMotorInit=false;
    //For encoder capture, drive encoders are zeroed after each drive step
    //The encoder counts are then written to a file on the phone
    private boolean loggingActive = true;




    @Override
    public void runOpMode(){
        //Prepare the gyro in the Expansion hub
        if (!skipMotorInit) initializeImu();

        //Create an array for drive motors
        ArrayList<DcMotor> motorList= new ArrayList<>();
        if (!skipMotorInit) initializeDriveMotors(motorList, false);

        waitForStart();

        for(int i = 0; i<4; i++){
            performDriveStep(0, 1, 0, 1500, motorList);

            twistToAngle(180, 0.8, motorList);
        }

    }
}
