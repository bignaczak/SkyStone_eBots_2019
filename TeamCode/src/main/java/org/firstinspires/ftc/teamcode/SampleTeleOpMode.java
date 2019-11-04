package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@TeleOp
@Disabled
public class SampleTeleOpMode extends eBotsOpMode2019 {

@Override
    public void runOpMode(){

    ArrayList<DcMotor> motorList= new ArrayList<>();

    if (motorList.size()>1) motorList.clear();  //Make sure there aren't any items in the list
    initializeDriveMotors(motorList, true);

    telemetry.addData("Status", "Motor Power Set, Ready");
    telemetry.update();


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
    final double fineAdjustMaxReduction = 0.7; //Don't allow drive to be fully negated
    boolean fineAdjustOn = false;               //Flag if fine adjust is activated
    boolean speedBoostOn = false;               //Maximize motor drive speeds if pressed
    final double motorThreshold=0.10;
    double spinScaleFactor = 0.4;

    //These values get calculated by calculateDriveVector function
    double[] driveValues = new double[4];  //Array of values for the motor power levels
    double maxValue;                        //Identify ax value in driveValues array


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
    }


}

}
