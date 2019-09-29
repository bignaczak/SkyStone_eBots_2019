package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp
public class testRobot_2mLED extends eBotsOpMode2019 {

    private Boolean skipMotorInit = false;

    //Setup for LEDs

    //Change the pattern every 10 seconds in AUTO mode.
    private final static int LED_PERIOD = 10;

    //Rate limit gamepad button presses to every 500ms.
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    @Override
    public void runOpMode() {
        //Prepare the gyro in the Expansion hub
        if (!skipMotorInit) initializeImu();

        //Create an array for drive motors
        ArrayList<DcMotor> motorList = new ArrayList<>();
        if (!skipMotorInit) initializeDriveMotors(motorList, true);

        initLEDs();

        //Initialize the variables that are being used in the main loop
        double spin;
        double driveX;
        double driveY;
        double r;       //length of radius for driveX and driveY
        double robotAngle;      //adjust of angle to account for mecanum drive
        double fineAdjust;
        final double fineAdjustThreshold = 0.3;    //Avoid trivial amounts of speed reduction with threshold value
        final double fineAdjustMaxReduction = 0.8; //Don't allow drive to be fully negated
        boolean fineAdjustOn = false;               //Flag if fine adjust is activated
        boolean speedBoostOn = false;               //Maximize motor drive speeds if pressed
        double[] driveValues = new double[4];
        double maxValue;

        double spinScaleFactor = 0.4;


        telemetry.addData("Status", "Set Motor Power Settings");
        telemetry.update();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();

        while (opModeIsActive()) {
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


            handleGamepad();

            if (displayKind == DisplayKind.AUTO) {
                doAutoDisplay();
            } else {
                /*
                 * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.
                 */
            }

        }
    }

    private void initLEDs(){
        displayKind = DisplayKind.AUTO;

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
     */
    private void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            telemetry.addData("gamepadRateLimit Not Expired",  gamepadRateLimit.hasExpired());
            telemetry.update();
            //return;
        }

        if (gamepad2.a) {
            setDisplayKind(DisplayKind.MANUAL);
            telemetry.addData("Button push", "A");
            telemetry.update();
            gamepadRateLimit.reset();
        } else if (gamepad2.b) {
            setDisplayKind(DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == DisplayKind.MANUAL) && (gamepad2.left_bumper)) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == DisplayKind.MANUAL) && (gamepad2.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        }
    }

    protected void setDisplayKind(DisplayKind newDisplayKind)
    {
        displayKind = newDisplayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
}
