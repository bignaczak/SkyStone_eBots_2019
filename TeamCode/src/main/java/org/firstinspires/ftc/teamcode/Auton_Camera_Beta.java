package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.ListIterator;
import java.util.concurrent.ThreadLocalRandom;

@Autonomous
public class Auton_Camera_Beta extends eBotsAuton2019 {


    /****************************************************************
     //******    CONFIGURATION PARAMETERS
     //***************************************************************/
    private Alliance alliance = Alliance.BLUE;
    private FieldSide fieldSide = FieldSide.QUARRY;
    private Speed speedConfig = Speed.MEDIUM;
    private GyroSetting gyroConfig = GyroSetting.EVERY_LOOP;
    private SoftStart softStartConfig = SoftStart.MEDIUM;
    private Accuracy accuracyConfig = Accuracy.LOOSE;



    @Override
    public void runOpMode(){
        /** APPLY CONFIG     **************/
        setGyroConfiguration(gyroConfig);      //Set gyro parameters and initialize
        setSpeedConfiguration(speedConfig);    //Set speed and PID gain limits
        setSoftStartConfig(softStartConfig);    //Set the soft start settings
        setAccuracyLimits(accuracyConfig);      //Set the accuracy limits
        setAllianceObjects(alliance);           //Create objects for Quarry Stones and Foundation
        simulateMotors = false;
        String logTag = "BTI_Auton_Camera_Beta";

        //***************************************************************
        //Initialize the drive motors
        //***************************************************************
        ArrayList<DcMotor> motorList= new ArrayList<>();
        //  Setup motors & encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        }else{
            initializeDriveMotors(motorList, true);
            initializeEncoderTrackers(motorList);           //actual encoders
            initializeManipMotors();
            initializeLimitSwitches();
        }

        //***************************************************************
        //Initialize the variables that are being used in the main loop
        //***************************************************************
        StopWatch rakeTimer = new StopWatch();
        StopWatch lifterTimer = new StopWatch();
        Boolean rakeBusy = false;
        Boolean clawBusy = false;
        Boolean lifterBusy = false;
        Boolean holdLifterPosition = false;

        //***************************************************************
        //  Read in the waypoints
        //***************************************************************

        wayPoses = new ArrayList<>();
        setWayPoses(wayPoses, alliance, fieldSide);
        ListIterator<Pose> iterator = wayPoses.listIterator();


        //  *********  INITIALIZE FOR FIRST PASS THROUGH LOOP   *****************
        //  Create currentPose, which is the tracked position from start to target
        //  This is a special type of pose that is intended to track the path of travel
        //  It has a targetPose, which is the intended destination
        //  It also has an error object which tracks it's status relative to targetPose
        TrackingPose currentPose;
        Pose startPose = null;
        Pose targetPose;

        //  Get the starting pose
        if (iterator.hasNext()) {
            startPose = iterator.next();
        }

        //Now get the target Pose
        if (iterator.hasNext()){
            targetPose = iterator.next();
        } else targetPose = startPose;

        //Construct the tracking pose
        currentPose = new TrackingPose(startPose, targetPose);
        Log.d(logTag, "First tracking pose created" + currentPose.toString());

        //  Capture the initial gyro offset for later use, it must be passed to each tracking pose
        if (useGyroForNavigation) {
            currentPose.setInitialGyroOffset(getGyroReadingDegrees());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }
        //This is called only once to document offset of gyro from field coordinate system
        Double initialGyroOffset = currentPose.getInitialGyroOffset();


        //***************************************************************
        //  Open up the webcam
        //***************************************************************
        prepWebcam();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        String loopMetrics = "Loop Initialized";
        writeOdometryTelemetry(loopMetrics, currentPose);
        waitForStart();

        StopWatch overallTime = new StopWatch();

        //Drop lifter to the limit switch
        raiseLifterToExtendArm();

        findLifterZero();

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************

        TrackingPose endPose = travelToNextPose(currentPose, motorList);

        Double currentPoseHeadingError;
        while(opModeIsActive() && iterator.hasNext()) {
            Log.d(logTag, "~~~~~~~~~~~Beginning of Loop for poses~~~~~~~~~~~~");

            //Perform actions specified in the targetPose of the path leg that was just completed
            executePostMoveActivity(currentPose);
            Log.d(logTag, "completed post move activity");

            //Correct heading angle if not within tolerance limits set in accuracyConfig
            correctHeading(currentPose, motorList);
            Log.d(logTag, "correcting angle");

            //Set ending of previous leg as the starting pose for new leg
            //  Position and pose are taken from the TrackingPose object returned by travelToNextPose
            Log.d(logTag, "Creating new start pose...");

            startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());

            //Some poses are considered "Scouting Poses" because Vuforia is being used to ID stones
            //These are denoted buy the postMoveActivity set to SCAN_FOR_SKYSTONE
            //If both SkyStones are found, skip the Scouting Poses
            boolean firstPassNextPose = true;   //To enter loop first pass
            Pose nextPose = null;        //Already checked hasNext in Loop
            Log.d(logTag, "next pose set to null");
            boolean skyStonesFound = (QuarryStone.getFoundSkyStoneCount() >= 2) ? true : false;
            boolean isScoutingPose = false;

            while(firstPassNextPose | (opModeIsActive() && skyStonesFound && isScoutingPose)){
                //Skip scouting poses if skystones already found
                if (nextPose != null) Log.d(logTag, "Skipping Scouting Pose " + nextPose.toString());
                if (iterator.hasNext()){
                    nextPose = iterator.next();
                    Log.d(logTag, "nextPose really assigned");
                } else {
                    //Last pose in list, take it and move on
                    break;
                }
                isScoutingPose = (nextPose.getPostMoveActivity() == Pose.PostMoveActivity.SCAN_FOR_SKYSTONE) ? true : false;
                firstPassNextPose = false;
            }
            Log.d(logTag, "Creating next tracking pose...");
            Log.d(logTag, "previousInitialGyroOffset: " + initialGyroOffset);
            currentPose = new TrackingPose(startPose, nextPose, initialGyroOffset);
            Log.d(logTag, "...completed");
            //Must set the initialGryOffset
            //This routine first sets the heading and then sets the initialGyroOffset
            //This is likely redundant and can be replaced with just a call to setInitialGyroOffset
            //Todo: Verify that copying the initialGyroOffset works and delete setInitialOffset...
            //currentPose.copyInitialGyroOffsetBetweenLegs(initialGyroOffset);
            //setInitialOffsetForTrackingPose(imu, currentPose, initialGyroOffset);

            Log.d("BTI_runOpMode", "~~~~~~~~~~~~~Starting Leg ");
            endPose = travelToNextPose(currentPose, motorList);

        }
        //Perform actions specified in the targetPose for the final leg
        executePostMoveActivity(currentPose);
        Log.d(logTag, "completed post move activity");

        Log.d(logTag, "Final pose reached, Correcting heading...");
        for (QuarryStone stone: QuarryStone.getQuarryStones()) {
            Log.d(logTag, "Writing stone data " + stone.toString());
        }


        correctHeading(currentPose, motorList);

        deliverSkyStonesToBuildingZone(endPose, motorList);

        //  Verify all drive motors are stopped
        if(!simulateMotors) stopMotors(motorList);

        Log.d(logTag, "Closing camera");

        //  Close vuforia webcam interface
        if (tfod != null) {
            tfod.shutdown();
        }
        Log.d(logTag, "Writing final telemetry");

        //Stop and hold telemetry for a while
        Log.d(logTag, "Total time: " + overallTime.toString());
        Log.d(logTag, "Total Stones in quarry: " + QuarryStone.getQuarryStones().size());
        Log.d(logTag, "Skystones found: " + QuarryStone.getFoundSkyStoneCount());
        StopWatch telemTimer = new StopWatch();
        telemetry.clear();
        telemetry.addData("Overall Time", overallTime.toString());
        telemetry.addData("SkyStones Found", QuarryStone.getFoundSkyStoneCount());
        for (QuarryStone stone: QuarryStone.getQuarryStones()) {
            Log.d(logTag, "Writing stone data " + stone.toString());

            telemetry.addLine(stone.toString());
        }
        telemetry.update();
        while (opModeIsActive() && telemTimer.getElapsedTimeMillis()<5000){
            //  Just hold here
        }
    }

    private void autoGrabBlockWithMovement(ArrayList<DcMotor> motorList){
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
    protected void autoGrabBlockNoMovement(ArrayList<DcMotor> motorList){
        /**
         * Automated routine to grab a block
         * 1) raise the lifter
         * 2) lower lifter while moving grabber wheel
         * 3) stop wheel and lift for travel
         */

        //  1) raise the lifter
        moveLifterToGrabHeight();

        //  2) Grab Stone
        grabStone();

        //  3) stop wheel and lift for travel
        liftStoneForTravel();

    }

    private void moveLifterToGrabHeight(){
        int lifterHeightApproach = blockHeightClicks;
        long timeout = 1000L;
        StopWatch timer = new StopWatch();
        int currentLifterPositionError;


        //  Raise the lifter to just above stone height
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
    }

    private void grabStone(){
        //  lower lifter while moving grabber wheel
        //  don't use run to position mode for this operation

        final Double rollerGripperSpeed = -0.7;
        Long pulseTimeOut = 750L;
        Long timeout = 2000L;
        StopWatch timer = new StopWatch();


        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setPower(0.3);   //
        timer.startTimer();

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
    }

    private void liftStoneForTravel(){
        //  Stop wheel and lift for travel
        int lifterHeightDrive = -150;

        rollerGripper.setPower(0.0);
        lifter.setTargetPosition(lifterHeightDrive);
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

    private void autoReleaseBlock(ArrayList<DcMotor> motorList){
        /**
         * 1) Lift Arm
         * 2) Pulse rollerGripper motor to release block
         */

        Integer lifterAutoReleaseIncrement = blockHeightClicks;
        final Double rollerGripperSpeed = 0.35;  //slow release speed
        final Double liftPowerLevelRelease = 0.25;
        Double rollerGripperPulseSpeed;
        Integer currentLifterPositionError;
        Long pulseTimeOut = 500L;
        Long timeout = 1400L;

        StopWatch timer = new StopWatch();
        timer.startTimer();


        lifterPosition = lifter.getCurrentPosition() + lifterAutoReleaseIncrement;
        lifter.setTargetPosition(lifterPosition);
        lifter.setPower(liftPowerLevelRelease);  //Try and raise slowly
        Boolean gripperPowerOn = true;
        currentLifterPositionError = lifterPosition - lifter.getCurrentPosition();

        while (opModeIsActive() && Math.abs(currentLifterPositionError) > 50
                && timer.getElapsedTimeMillis() < timeout){
            rollerGripper.setPower(rollerGripperSpeed);
            /*
            if (gripperPowerOn) {
                rollerGripperPulseSpeed = rollerGripperSpeed;
            } else rollerGripperPulseSpeed = 0.0;
            pulseRollerGripper(pulseTimeOut, rollerGripperPulseSpeed);
            */
            writeAutoGrabTelemetry("Releasing Block");

            currentLifterPositionError = lifterPosition - lifter.getCurrentPosition();
            gripperPowerOn = !gripperPowerOn;
        }

        //  Set power level back
        lifter.setPower(lifterPowerLevel);


    }


    private void deliverSkyStonesToBuildingZone(TrackingPose endPose, ArrayList<DcMotor> motorList){
        /**
         * Drive from current location to the next SkyStone
         */

        String logTag = "BTI_deliverSkyStone...";
        //Retrieve a skystone from those observed.  If not observed, select random
        QuarryStone skyStone = getTargetSkyStone();

        //Travel to the SkyStone
        Log.d(logTag, "Getting tracking pose to skystone");
        TrackingPose currentPose = getTrackingPoseToSkyStone(skyStone, endPose);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving to skyStone");
        endPose = travelToNextPose(currentPose, motorList);     //Actually drive
        Log.d(logTag, "SkyStoe travel completed, Location: " + currentPose.toString());

        //**  Now perform the autoGrab function
        autoGrabBlockNoMovement(motorList);

        //Travel to Foundation
        Log.d(logTag, "Getting tracking pose to foundation");
        currentPose = getTrackingPoseToFoundation(endPose);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving to foundation");
        endPose = travelToNextPose(currentPose, motorList);     //Actually drive
        Log.d(logTag, "Foundation travel completed, Location: " + currentPose.toString());
    }

    private QuarryStone getTargetSkyStone(){
        /**
         * Returns a skyStone for transport to Building Zone
         * If none were observed, it will pick a random one
         */

        String logTag = "BTI_getTargetSkyStone";
        //Pick up the SkyStone
        Log.d(logTag, "Retrieving skyStone coordinates, " + QuarryStone.getFoundSkyStoneCount()
                + " skystones found");

        //todo: fix this so a second skystone can be pursued, currently will pick same one
        ArrayList<QuarryStone> quarrystones = QuarryStone.getSkyStones();
        ListIterator<QuarryStone> quarryStoneListIterator = quarrystones.listIterator();
        QuarryStone skyStone;
        QuarryStone.StoneLocation skyStoneLocation;

        //If skystone is found, then retrieve the first item on the list,
        //otherwise pick a random number
        if (quarryStoneListIterator.hasNext()){
            skyStone = quarryStoneListIterator.next();
            skyStoneLocation = skyStone.getStoneLocation();
            Log.d(logTag, "Going for stone: " + skyStone.toString());
            Log.d(logTag, "Removing from list, currently with " + quarrystones.size() + " items");
            quarryStoneListIterator.remove();
            Log.d(logTag, "Item removed, now with " + quarrystones.size() + " items");
        } else {
            int randomNum = ThreadLocalRandom.current().nextInt(1, 6);  //note, excludes block against wall
            skyStoneLocation = QuarryStone.StoneLocation.getStoneLocation(randomNum);
            skyStone = QuarryStone.getQuarryStone(skyStoneLocation);
            Log.d(logTag, "Going for random stone: " + skyStone.toString());
        }
        return skyStone;
    }

    private TrackingPose getTrackingPoseToSkyStone(QuarryStone skyStone, TrackingPose endPose){
        String logTag = "BTI_getTrack.SkyStone.";
        double stoneX = skyStone.getStoneLocation().getXStone();   //retrieve x dimension
        double stoneY;
        if(alliance == Alliance.RED){
            stoneY = skyStone.getStoneLocation().getYStone()- offsetDistance;  //retrieve y dimension
        } else {
            stoneY = skyStone.getStoneLocation().getYStone()+ offsetDistance;  //retrieve y dimension
        }
        double stoneHeading = endPose.getTargetPose().getHeading();  //retrieve heading

        //Create the beginning and end poses for the move
        Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        Pose skyStonePose = new Pose(stoneX,   stoneY , stoneHeading);
        Log.d(logTag, "Creating tracking pose to SkyStone " + skyStone.toString());
        //Create the tracking pose
        return new TrackingPose(startPose, skyStonePose, endPose.getInitialGyroOffset());
    }

    protected TrackingPose getTrackingPoseToFoundation(TrackingPose endPose){
        String logTag = "BTI_getTracking.Found.";
        Log.d(logTag, "Creating tracking pose to Foundation...");

        //Create the beginning and end poses for the move

        Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        Pose foundationPlate = foundation.getSkyStoneDumpingPose(offsetDistance);
        Log.d(logTag, "Creating tracking pose to Foundation COMPLETED!");
        return new TrackingPose(startPose, foundationPlate,endPose.getInitialGyroOffset());
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
