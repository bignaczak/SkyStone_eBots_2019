package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

import static java.lang.String.format;

@Disabled
@TeleOp
public class Teleop_Competition extends LinearOpMode {
    //Drive Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Manipulation Motors
    private DcMotor armAngleMotor;
    private DcMotor armExtensionMotor;
    private DcMotor collectMotor;
    private DcMotor latchMotor;


    //  DEFINE CONSTANTS FOR THE ROBOT
    //  THESE ARE ALL POSITIONS ASSUMED THE ROBOT IS COMPLETELY FOLDED PRIOR TO START OF AUTON
    final static int ARM_ANGLE_COLLECT_POSITION = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_COLLECT_POSITION = -10000;  //TEST POSITION
    final static int ARM_ANGLE_TRAVEL_POSITION = -5600;
    final static int ARM_ANGLE_SCORE_POSITION = -1800;    //MUST VERIFY
    //final static int ARM_ANGLE_DUMP_POSITION = -7300;       //Was -9500

    //These are constants used to define counts per revolution of NEVEREST motors with encoders
    static final int NEVEREST_60_CPR = 1680;
    static final int NEVEREST_40_CPR = 1120;
    static final int NEVEREST_20_CPR = 560;

    //final int ARM_EXTENSION_COLLECTION_POSITION = 27800;
    final static int ARM_EXTENSION_COLLECTION_POSITION = -57500;  //test position
    final static int ARM_EXTENSION_TRAVEL_POSITIION = -27800;
    final static int ARM_EXTENSION_DUMP_POSITION = -27800;

    final static int LATCH_DEPLOY_POSITION = -13200;        //13900 is a little too high
    //13500 is a little high for our practice lander
    final static int LATCH_DRIVE_POSITION = -5900;          //5900 is good, could be a little higher
    final static int LATCH_ENGAGE_POSITION = -11000;
    final static int LATCH_RISEUP_POSITION = -2500;

    //MOTOR PROTECTION ENCODER
    final static int ARM_ANGLE_LIMIT = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_LIMIT = -10000;  //TEST POSITION
    final static int ARM_EXTENSION_LIMIT = -57500;
    final static int LATCH_LIMIT = -14000;

    final static String VUFORIA_KEY = "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";

    final static long SAMPLING_DRIVE_TIME = 1400;
    final static long SAMPLING_EXTRA_DRIVE_TIME = 500;
    final static double SAMPLING_DRIVE_COMPONENT = 0.85;
    final static double SAMPLE_ALIGNMENT_TWIST_ANGLE = 30;

    final static long DEPOT_DRIVE_TIME = 1100;
    final static long DEPOT_EXTRA_DRIVE_TIME = 0;
    final static double DEPOT_DRIVE_COMPONENT = .4;
    final static long CRATER_DRIVE_TIME = 2800;  //was 2500 for first successful sample
    final static long EXTRA_CRATER_DRIVE_TIME = 700;
    //****************************************************************
    //END CONSTANTS


    //This teleOp is assumed to run immediately after Auton
    // So the zero position must be offset based on the final positions (assuming correct ending position)
    // The armAngle and armExtension variables get assigned right before start button is pushed
    // in the function zeroArmEncodersOnTheFly()
    static int armAngleEffectiveZeroPoint;
    static int armAngleEffectiveLimit;
    static int armAngleEffectiveTravelPositiion;  //This is the assumed starting point
    static int armAngleEffectiveScorePosition;

    static int armExtensionEffectiveZeroPoint;
    static int armExtensionEffectiveLimit;  //This will be a small diff

    //static int latchEffectiveZeroPoint = -LATCH_DRIVE_POSITION;
    //static int latchEffectiveLimit = LATCH_LIMIT - LATCH_DRIVE_POSITION;
    //static int latchEffectiveEngagePosition = LATCH_ENGAGE_POSITION - LATCH_DRIVE_POSITION;
    //static int latchEffectiveRiseupPosition = LATCH_RISEUP_POSITION - LATCH_DRIVE_POSITION;
    static int latchEffectiveZeroPoint = -LATCH_DEPLOY_POSITION;
    static int latchEffectiveLimit = LATCH_LIMIT - LATCH_DEPLOY_POSITION;
    static int latchEffectiveEngagePosition = LATCH_ENGAGE_POSITION - LATCH_DEPLOY_POSITION;
    static int latchEffectiveRiseupPosition = LATCH_RISEUP_POSITION - LATCH_DEPLOY_POSITION;


    private void calculateDriveVector(double driveMagnitude, double robotAngle, double spin, double[] outputArray){
        //Create an array of the drive values
        //[Element] --> Wheel
        //  [0] --> Front Left
        //  [1] --> Front Right
        //  [2] --> Back Left
        //  [3] --> Back Right
        outputArray[0] = driveMagnitude * Math.cos(robotAngle) + spin;
        outputArray[1] = driveMagnitude * Math.sin(robotAngle) - spin;
        outputArray[2] = driveMagnitude * Math.sin(robotAngle) + spin;
        outputArray[3] = driveMagnitude * Math.cos(robotAngle) - spin;

        //Now capture the max drive value from the array
        double maxValue = findMaxAbsValue(outputArray);

        //If any of the values exceed 1, then all drive values must be scaled
        //Divide all drive values by the max value to achieve a new max value of 1
        if (maxValue > 1) scaleDrive(1/maxValue, outputArray);
    }

    private void scaleDrive (double scaleFactor, double[] driveArray){
        for (int i=0; i<driveArray.length; i++) {
            driveArray[i] *= scaleFactor;
        }
    }

    private double findMaxAbsValue(double[] array) {
        //Now capture the max drive value from the array
        double maxValue = Math.abs(array[0]);
        for (int i = 1; i < array.length; i++){
            if(Math.abs(array[i]) > maxValue) maxValue = Math.abs(array[i]);
        }
        return maxValue;
    }

    private void zeroArmEncodersOnTheFly(int newArmAngleResetPoint, int newArmExtensionResetPoint){
        //This zeros the encoders for the armAngle and armExtension motors
        //It is intended to remove the need to use the encoder override continuously
        //Which may happen if auton doesn't complete successfully
        //the inputs should be one of the constants listed in the class variables
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Assume reset happens when arm fully extended

        //To figure out these positions, take the orignal sweep angle minus new position
        armAngleEffectiveZeroPoint = -newArmAngleResetPoint;  //zero sweep angle
        armAngleEffectiveLimit = ARM_ANGLE_LIMIT - newArmAngleResetPoint;
        armAngleEffectiveTravelPositiion = ARM_ANGLE_TRAVEL_POSITION - newArmAngleResetPoint;
        armAngleEffectiveScorePosition = ARM_ANGLE_SCORE_POSITION - newArmAngleResetPoint;

        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionEffectiveZeroPoint = -newArmExtensionResetPoint;
        armExtensionEffectiveLimit = ARM_EXTENSION_LIMIT - newArmExtensionResetPoint;

        armAngleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void zeroDriveEncodersOnTheFly(List<DcMotor> motors){
        for (DcMotor m: motors){
            m.setPower(0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //if needed
        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void runOpMode(){
        //imu = hardwareMap.get(Gyroscope.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        //reverse direction for opposite side motors
        //This is needed based on how the motors are mounted on the robot
        //Clockwise vs. Counter Clockwise being forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Set Motor Power Settings");
        telemetry.update();

        //Create an array of motors with their associated Power Setting
        List<DcMotor> motorList= new ArrayList<>();
        motorList.add(frontLeft);
        motorList.add(frontRight);
        motorList.add(backLeft);
        motorList.add(backRight);

        for (DcMotor m: motorList){
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Status", "Motor Power Set, Ready");
        telemetry.update();

        //Initialize the variables that are being used in the main loop
        double spin;
        double driveX;
        double driveY;
        double r;       //length of radius for driveX and driveY
        double robotAngle;      //adjust of angle to account for mecanum drive
        double fineAdjust;
        final double fineAdjustThreshold = 0.3;    //Avoid trivial amounts of speed reduction with threshold value
        final double fineAdjustMaxReduction = 0.5; //Don't allow drive to be fully negated
        boolean fineAdjustOn = false;               //Flag if fine adjust is activated
        boolean speedBoostOn = false;               //Maximize motor drive speeds if pressed
        double[] driveValues = new double[4];
        double maxValue;
        long autoMoveStart = System.nanoTime()/1000000;
        long autoMoveTimeLimit = 6000;

        //Initialize motors for manipulator
        armAngleMotor = hardwareMap.get(DcMotor.class, "armAngleMotor");
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armAngleMotor.setDirection(DcMotor.Direction.REVERSE);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "armExtensionMotor");
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");

        latchMotor = hardwareMap.get(DcMotor.class, "latchMotor");
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        latchMotor.setDirection(DcMotor.Direction.REVERSE);

        int countsPerRevolution = NEVEREST_20_CPR;  //Output counts per revolution of Output Shaft (cpr): 1120
        double degreesToEncoderCounts = countsPerRevolution/360.0;
        int targetAngle;
        int targetEncoderCounts;
        double rightTrigger;
        double leftTrigger;
        double latchDrive;
        double armAngleDrive;
        double armExtensionDrive;
        final double motorThreshold=0.10;
        boolean encoderSafetyOverride = false;
        final int encoderCountSafetyBuffer = 1000;
        final double motorThrottleDownSpeed = 0.5;
        double spinScaleFactor = 0.4;

        zeroArmEncodersOnTheFly(ARM_ANGLE_TRAVEL_POSITION, ARM_EXTENSION_COLLECTION_POSITION);

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();

        //run until the end of the match (driver presses STOP)
        while(opModeIsActive()){

            //This opMode set routines for some of the manip motors to move to position
            //This requires transitioning the motor between different operating modes
            //At the beginning of each loop, look at the mode for the manip motors
            //and if not current busy, change mode back to run_without_encoders
            if(!latchMotor.isBusy()){
                if (latchMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                    latchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            if(!armAngleMotor.isBusy()){
                if (armAngleMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                    armAngleMotor.setPower(0);
                    armAngleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }else{
                //Sometimes the armAngle motor gets stuck in the Busy mode
                //So if it is busy but has taken too much time, reset the mode
                if((System.nanoTime() / 1000000) > (autoMoveStart+autoMoveTimeLimit)){
                    armAngleMotor.setPower(0);
                    armAngleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }
                if (Math.abs(armAngleMotor.getCurrentPosition() - armAngleMotor.getTargetPosition()) < encoderCountSafetyBuffer){
                    //If getting close to the target, turn down power
                    armAngleMotor.setPower(motorThrottleDownSpeed);
                }
            }


            //GAMEPAD1 INPUTS
            //----------------------------------------
            //Get the drive inputs from the controller
            //  [LEFT STICK]   --> Direction and Speed
            //  [RIGHT STICK]  --> X Direction dictates spin rate to rotate about robot center
            //  [LEFT TRIGGER] --> Variable reduction in robot speed to allow for fine position adjustment
            //  [RIGHT BUMPER] --> Speed boost, maximized motor drive speed

            driveX = gamepad1.left_stick_x;        //Motion
            driveY = -gamepad1.left_stick_y;         //Motion
            spin = gamepad1.right_stick_x * spinScaleFactor; //This is used to determine how to spin the robot
            fineAdjust = gamepad1.left_trigger;     //Pull to slow motion
            speedBoostOn = gamepad1.right_bumper;   //Push to maximize motor drives

            //r gives the left stick's offset from 0 position by calculating hypotenuse of x and y offset
            r = Math.hypot(driveX, driveY );

            //Robot angle calculates the angle (in radians) and then subtracts pi/4 (45 degrees) from it
            //The 45 degree shift aligns the mecanum vectors for drive
            robotAngle = Math.atan2(driveY, driveX) - Math.PI / 4;
            calculateDriveVector(r, robotAngle, spin, driveValues);     //Calculate motor drive speeds
            //Math.atan2
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
            }
            else{
                fineAdjustOn = false;
                fineAdjust=0;
            }
            fineAdjust = 1 - fineAdjust;

            if (fineAdjustOn) scaleDrive(fineAdjust, driveValues);    //Apply Fine Adjust


            //Now maximize speed by applying a speed boost
            //The drive calculation sometimes doesn't set the peak drive to 1, this corrects that
            if (!fineAdjustOn & speedBoostOn){      //Fine Adjust mode takes precedent over speed boost
                maxValue = findMaxAbsValue(driveValues);  //See what the max drive value is set to
                if (maxValue < 1 & maxValue > 0) scaleDrive(1/maxValue, driveValues);  //If max value isn't 1, Scale the values up so max is 1
            }

            //Now actually assign the calculated drive values to the motors in motorList
            Log.d("BTI_BeforeSet", printActualMotorPowers(motorList));
            Log.d("BTI_Calc", printMotorDriveInstructions(motorList, driveValues));
            int i=0;
            for (DcMotor m: motorList){
                m.setPower(driveValues[i]);
                i++;
            }
            Log.d("BTI_AfterSet", printActualMotorPowers(motorList));

            //GAMEPAD 2 INPUTS
            //  Y Button        --> arm angle positive direction
            //  A Button        --> arm angle negative direction
            //  Right Bumper    --> arm angle in dump position
            //  dpad UP         --> arm extension positive direction
            //  dpad DOWN       --> arm extension negative direction
            //  Right Trigger   --> collection motors positive
            //  Left Trigger    --> collection motor negative
            //  Left Stick      --> move latch up/down
            //  Right Stick     --> arm angle up/down
            //  Left Bumper     --> encoder override

            if (gamepad2.left_bumper) {
                encoderSafetyOverride = true;
            } else {
                encoderSafetyOverride = false;
            }

            //Manipulator controls

            //Control arm angle with right stick
            armAngleDrive = gamepad2.right_stick_y;

            //Note that the armAngleMotor is setup so
            // Extending to collect counts NEGATIVE encoder clicks
            // Retracting into body counts POSITIVE encoder clicks
            // If the joystick is moved more than threshold
            // if trying to extend, make sure that the position is within encoder safety limit
            // This teleOp is assumed to run immediately after Auton
            // So the zero position must be offset a bit
            if (armAngleDrive < -motorThreshold & armAngleMotor.getCurrentPosition() > armAngleEffectiveLimit
                    | armAngleDrive < -motorThreshold & encoderSafetyOverride){
                    //Pushing up on the stick gives a negative value
                    //No need to adjust armAngleDrive
                    //It is equal to the value bounded by threshold and 1
            } else if (armAngleDrive > motorThreshold & armAngleMotor.getCurrentPosition() < armAngleEffectiveZeroPoint
                    | armAngleDrive > motorThreshold & encoderSafetyOverride){
                //if trying to retract, make sure that the position is above zero
                    //No need to adjust armAngleDrive
                    //It is equal to the value bounded by -threshold and -1
            }else{
                    //Set armAngleDrive to zero
                    armAngleDrive = 0;
            }

            if (!armAngleMotor.isBusy()) {
                if(Math.abs(armAngleMotor.getCurrentPosition() - armAngleEffectiveLimit) < encoderCountSafetyBuffer
                        | Math.abs(armAngleMotor.getCurrentPosition() - armAngleEffectiveZeroPoint) < encoderCountSafetyBuffer){
                    //if close to one of the limits, reduce power to improve accuracy
                    if (armAngleDrive <= -motorThrottleDownSpeed) {
                        armAngleDrive = -motorThrottleDownSpeed;
                    } else if (armAngleDrive >= motorThrottleDownSpeed){
                        armAngleDrive = motorThrottleDownSpeed;
                    }
                }
                armAngleMotor.setPower(armAngleDrive);
            }


            //Control armExtension with dpad up and down
            // Note that the armExtension encoder counts DECREASE with extension
            //                        and encoder counts INCREASE with retraction
            // This is important for setting the encoder safety limits
            if ((gamepad2.dpad_up & armExtensionMotor.getCurrentPosition() > armExtensionEffectiveLimit)
                    | (gamepad2.dpad_up & encoderSafetyOverride)) {
                armExtensionDrive = -1;
                //armExtensionMotor.setPower(-1);
            } else if ((gamepad2.dpad_down & armExtensionMotor.getCurrentPosition() < armExtensionEffectiveZeroPoint)
                    | (gamepad2.dpad_down & encoderSafetyOverride)) {
                armExtensionDrive = 1;
                //armExtensionMotor.setPower(1);
            } else {
                armExtensionDrive = 0;
                //armExtensionMotor.setPower(0);
            }

            if(Math.abs(armExtensionMotor.getCurrentPosition() - armExtensionEffectiveLimit) < encoderCountSafetyBuffer
                    | Math.abs(armExtensionMotor.getCurrentPosition() - armExtensionEffectiveZeroPoint) < encoderCountSafetyBuffer){
                //if close to one of the limits, reduce power to improve accuracy
                if (armExtensionDrive <= -motorThrottleDownSpeed) {
                    armExtensionDrive = -motorThrottleDownSpeed;
                } else if (armExtensionDrive >= motorThrottleDownSpeed){
                    armExtensionDrive = motorThrottleDownSpeed;
                }
            }
            armExtensionMotor.setPower(armExtensionDrive);

            //Get input for the collector motors
            rightTrigger = gamepad2.right_trigger;
            leftTrigger=gamepad2.left_trigger;

            if (rightTrigger > motorThreshold) {
                collectMotor.setPower(rightTrigger);
            } else if (leftTrigger>motorThreshold) {
                collectMotor.setPower(-leftTrigger);
            }else {
                collectMotor.setPower(0);
            }

            //Allow the armAngle and armExtension motor encoders to be reset in case auton
            // doesn't fully complete
            // Arm should be down and fully extended when called
            if(gamepad2.left_bumper & gamepad2.right_bumper & gamepad2.x){
                zeroArmEncodersOnTheFly(ARM_ANGLE_COLLECT_POSITION, ARM_EXTENSION_COLLECTION_POSITION);
            }

            //Allow drive motor encoders to be zeroed on the fly
            if(gamepad2.left_bumper & gamepad2.right_bumper & gamepad2.dpad_left){
                zeroDriveEncodersOnTheFly(motorList);
            }

            //Below are present positions for the arm angle
            //*************************************************

            //Set armAngle to travel position
            //  RIGHT BUMPER + B
            if (gamepad2.right_bumper & gamepad2.b ) {  //& !armAngleMotor.isBusy()
                armAngleMotor.setPower(0);
                armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armAngleMotor.setTargetPosition(armAngleEffectiveTravelPositiion);
                armAngleMotor.setPower(1);
                //TODO:  Make timeout a function of starting position
                autoMoveTimeLimit = 3500;  //only allow 3.5 seconds for move
                autoMoveStart = System.nanoTime() / 1000000;  //Current time in milliseconds
            }

            //Set armAngle to score position
            //  RIGHT BUMPER + Y
            if (gamepad2.right_bumper & gamepad2.y ) {  //& !armAngleMotor.isBusy()
                armAngleMotor.setPower(0);
                armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armAngleMotor.setTargetPosition(armAngleEffectiveScorePosition);
                armAngleMotor.setPower(1);
                //TODO:  Make timeout a function of starting position
                autoMoveTimeLimit = 6000;  //only allow 6 seconds for move
                autoMoveStart = System.nanoTime() / 1000000;  //Current time in milliseconds

            }

            //Set armAngle to collect position
            //  RIGHT BUMPER + A
            if (gamepad2.right_bumper & gamepad2.a ) {  //& !armAngleMotor.isBusy()
                armAngleMotor.setPower(0);
                armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armAngleMotor.setTargetPosition(armAngleEffectiveLimit);
                armAngleMotor.setPower(1);
                //TODO:  Make timeout a function of starting position
                autoMoveTimeLimit = 6000;  //only allow 7 seconds for move
                autoMoveStart = System.nanoTime() / 1000000;  //Current time in milliseconds

            }
            //*************************************************
            // Note that the latch encoder counts INCREASES with extension (i.e. lowers robot)
            //                        and encoder counts DECREASES with retraction  (i.e. robot raises)
            // This is important for setting the encoder safety limits

            latchDrive = gamepad2.left_stick_y;
            //Note:  down in the y direction is POSITIVE signal
            if (latchDrive > motorThreshold & latchMotor.getCurrentPosition() < latchEffectiveZeroPoint
                    | latchDrive > motorThreshold & encoderSafetyOverride){
                    //Note:  pushing down on the stick lowers the are (raises robot)
                    //  This is changed from original behavior, motor direction has been reversed
                    //  Pushing down causes encoder count to INCREASE

            }else if(latchDrive < -motorThreshold & latchMotor.getCurrentPosition() > latchEffectiveLimit
                    | latchDrive < -motorThreshold & encoderSafetyOverride){
                    //Note:  pushing up on the stick has been changed to raise the arm (lowers robot)
                    //  Pushing up causes the encoder count to DECREASE
            } else {
                latchDrive = 0;
            }

            if (!latchMotor.isBusy()) {
                latchMotor.setPower(latchDrive);
            }

            //Below are preset routine to engage the latch and lift the robot
            //must push both bumpers and push in left stick button to activate
            //It engages the latch and then lifts robot
            //Note that this is blocking code, so all other functions stop
            if(gamepad2.left_bumper & gamepad2.right_bumper & gamepad2.left_stick_button){
                latchMotor.setPower(0);
                latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                latchMotor.setTargetPosition(latchEffectiveEngagePosition);
                latchMotor.setPower(1);
                while (latchMotor.isBusy()){
                    telemetry.addData("Status", "Engaging Latch");
                    telemetry.addData("latchMotor Position", latchMotor.getCurrentPosition());
                    telemetry.update();

                }
                //wait for latch to engage
                latchMotor.setPower(0);
                latchMotor.setTargetPosition(latchEffectiveRiseupPosition);
                latchMotor.setPower(1);
                while (latchMotor.isBusy()){
                    telemetry.addData("latchMotor Position", latchMotor.getCurrentPosition());
                    telemetry.addData("Status", "Lifting");
                    telemetry.update();

                    //wait for robot to be lifted
                }
                latchMotor.setPower(0);
                telemetry.addData("latchMotor Position", latchMotor.getCurrentPosition());
                telemetry.addData("Status", "Latching Complete!!");
                telemetry.update();


                //Now put back into original power mode
                latchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //telemetry.addData("driveX", driveX);
            //telemetry.addData("driveY", driveY);
            //telemetry.addData("spin", spin);
            //telemetry.addData("encoder override", gamepad2.left_bumper);
            //telemetry.addData("right bumper", gamepad2.right_bumper);
            //telemetry.addData("right stick y", gamepad2.right_stick_y);
            telemetry.addData("armAngle busy", armAngleMotor.isBusy());
            telemetry.addData("armAngle mode", armAngleMotor.getMode());
            telemetry.addData("run without encoders", DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("armAngleMotor Position", armAngleMotor.getCurrentPosition());
            telemetry.addData("armExtensionMotor Position", armExtensionMotor.getCurrentPosition());
            telemetry.addData("latchMotor Position", latchMotor.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.addData("frontLeft Position", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight Position", frontRight.getCurrentPosition());
            telemetry.addData("backLeft Position", backLeft.getCurrentPosition());
            telemetry.addData("backRight Position", backRight.getCurrentPosition());

            telemetry.update();
        }
    }

    public String printMotorDriveInstructions(List<DcMotor> motorList, double[] driveValues){
        StringBuilder outputString = new StringBuilder();
        for(Integer i=0; i<motorList.size(); i++){
            if(i>0) outputString.append(" | ");
            outputString.append("Motor_" + i.toString());
            outputString.append(" Port" + motorList.get(i).getPortNumber());
            outputString.append(" [" + format("%.2f", driveValues[i]) + "]");
        }
        return outputString.toString();
    }

    public String printActualMotorPowers(List<DcMotor> motorList){
        StringBuilder outputString = new StringBuilder();
        for(Integer i=0; i<motorList.size(); i++){
            if(i>0) outputString.append(" | ");
            outputString.append("Motor_" + i.toString());
            outputString.append(" Actual Power:" + motorList.get(i).getPortNumber());
            outputString.append(" [" + format("%.2f", motorList.get(i).getPower()) + "]");
        }
        return outputString.toString();
    }

}
