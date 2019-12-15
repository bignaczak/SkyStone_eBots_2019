package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

import static org.firstinspires.ftc.teamcode.eBotsMotionController.defendPosition;
import static org.firstinspires.ftc.teamcode.eBotsMotionController.moveToTargetPose;

@Autonomous
public class Auton_STATES_V2 extends eBotsAuton2019 {


    @Override
    public void runOpMode(){
        boolean debugOn = true;
        String logTag = "BTI_E3_UsingConfig";

        //LinearOpMode opMode = (LinearOpMode) this;

        //***************************************************************
        // ******    CONFIGURATION PARAMETERS
        // ***************************************************************/
        speedConfig = Speed.SLOW;
        gyroConfig = GyroSetting.INFREQUENT;
        softStartConfig = SoftStart.MEDIUM;
        accuracyConfig = Accuracy.STANDARD;


        //Apply the configurations
        importConfigurationFile();


        /** APPLY CONFIG     **************/
        setGyroConfiguration(gyroConfig);      //Set gyro parameters and initialize
        setSpeedConfiguration(speedConfig);    //Set speed and PID gain limits
        setSoftStartConfig(softStartConfig);    //Set the soft start settings
        setAccuracyLimits(accuracyConfig);      //Set the accuracy limits
        setAllianceObjects();           //Create objects for Quarry Stones and Foundation

        simulateMotors = false;

        Log.d(logTag, "Starting OpMode...");

        //  Setup motors & encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        } else{
            initializeDriveMotors(true);
            initializeEncoderTrackers(getDriveMotors());
            initializeManipMotors();
            rake3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rake3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            initializeLimitSwitches();          //limit switches
            initializeDistanceSensors();        //distance sensors
        }

        if (debugOn) Log.d(logTag, "Encoders initialized " + EncoderTracker.getEncoderTrackerCount() + " found");


        //  Create the first TrackingPose
        //  Note, setting the initialGyroOffset is critical
        TrackingPose trackingPose = getFirstTrackingPose();


        //***************************************************************
        // Initiate the concurrent process to scan the quarry
        //***************************************************************
        ObserveQuarryDuringWait observeQuarryDuringWait;
        Thread thread;
        if (fieldSide == FieldSide.QUARRY && delayedStart == DelayedStart.NO) {
            //***************************************************************
            //  Open up the webcam
            //***************************************************************
            prepWebcam();

            observeQuarryDuringWait = new ObserveQuarryDuringWait();
            thread = new Thread(observeQuarryDuringWait);
            thread.start();

        } else {
            observeQuarryDuringWait = null;
            thread = null;
        }

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        endInitTelemetry();
        waitForStart();

        if (fieldSide == FieldSide.QUARRY && delayedStart == DelayedStart.NO) {
            observeQuarryDuringWait.keepRunning = false;
            thread.interrupt();
        }

        overallTime = new StopWatch();




        if (delayedStart == DelayedStart.YES){
            executeDelayedRoutine(trackingPose);
        }else if(fieldSide == FieldSide.QUARRY){
            executeQuarryRoutine(trackingPose);
        } else {
            executeFoundationRoutine(trackingPose);
        }


        //  Verify all drive motors are stopped
        if(!simulateMotors) stopMotors();

        if (debugOn) Log.d(logTag, "Closing camera");

        //  Close vuforia webcam interface
        if (fieldSide == FieldSide.QUARRY && delayedStart == DelayedStart.NO && tfod != null) {
            tfod.shutdown();
        }

        telemetry.clear();
        while (opModeIsActive() && overallTime.getElapsedTimeMillis() < 5000){
            telemetry.addLine("Execution Continued");
            telemetry.addData("elapsed millis", overallTime.getElapsedTimeMillis());
            telemetry.addData("SkyStones Found:", QuarryStone.getFoundSkyStoneCount());
            telemetry.update();
        }
        if (debugOn) Log.d(logTag, "OpMode Complete, Overall Time: " + overallTime.toString());
    }
    private void executeFoundationRoutine(TrackingPose trackingPose){
        String logTag = "BTI_executeFound";
        boolean debugOn = true;
        StopWatch overallTimer = new StopWatch();
        //Travel first leg
        Integer travelLegCount = 1;   //Positions 0 and 1 already consumed in first TrackingPose
        ListIterator<Pose> listIterator = wayPoses.listIterator();
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Starting LegStarting Leg " + travelLegCount.toString());
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************

        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "Leg " + travelLegCount + " completed, new position " + trackingPose.toString());
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        travelLegCount++;

        while(opModeIsActive() && listIterator.hasNext()) {

            //Correct heading angle if not within tolerance limits set in accuracyConfig
            correctHeading(trackingPose);
            //Perform actions specified in the targetPose of the path leg that was just completed
            executePostMoveActivity(trackingPose, alliance);

            //Set ending of previous leg as the starting pose for new leg
            //  Position and pose are taken from the TrackingPose object returned by travelToNextPose
            //  TODO:  This is not necessary, should just replace the targetPose of the existing trackingPose
            //   instead, eliminating need to copy initialGyroOffset

            Pose targetPose = listIterator.next();
            trackingPose.setTargetPose(targetPose);
            //Remove the item from the wayPose list
            listIterator.remove();
            //Must set the initialGryOffset

            if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Starting Leg " + travelLegCount.toString()
                    + " to " + trackingPose.getTargetPose().toString());
            if (debugOn) Log.d(logTag, "Start Position " + trackingPose.toString());
            moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);

            if (debugOn) Log.d(logTag, "Leg " + travelLegCount + " completed, new position " + trackingPose.toString());
            if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

            travelLegCount++;
        }

        executePostMoveActivity(trackingPose, alliance);
        yoyo.setPower(YOYO_EXTEND);
        StopWatch yoyoTimer = new StopWatch();
        long yoyoTimeout = 5000L;
        while (opModeIsActive() && yoyoTimer.getElapsedTimeMillis() < yoyoTimeout){
            //Let it extend
        }

        long holdDuration = 29000L - overallTimer.getElapsedTimeMillis();
        Pose targetPose = new Pose (trackingPose.getX(), trackingPose.getY(), trackingPose.getHeading());
        trackingPose.setTargetPose(targetPose);
        if(holdDuration>0) defendPosition(trackingPose, holdDuration, speedConfig, gyroConfig, accuracyConfig, softStartConfig, imu, telemetry);

        yoyo.setPower(YOYO_STOP);

        if (debugOn) Log.d(logTag, "All Legs Completed, current Position " + trackingPose.toString());
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());
    }



    private void executeQuarryRoutine(TrackingPose trackingPose){
        boolean debugOn = true;
        String logTag = "BTI_executeQuarry";
        Pose targetPose;

        if (debugOn) Log.d(logTag, "Starting executeQuarryRoutine...");

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************
        //Extend Arm, find zero, and set lifter to stone grab height
        raiseLifterToExtendArm();

        //  hopefully the skystone was identified during the init period
        //  If not, move to a second vantage point for analysis
        if (QuarryStone.getFoundSkyStoneCount() == 0) {
            if (QuarryStone.getCountObservedStones() <= 1) {
                surveilQuarryFromSecondPosition(trackingPose, overallTime);
            } else {
                ArrayList<QuarryStone> observedStones = QuarryStone.getObservedStones();
                if (observedStones.size() > 1) {
                    determineSkyStonePattern(observedStones.get(0), observedStones.get(1));
                }
            }
        } else {
            Log.d(logTag, "Stones are already found!!");
        }

        //Pick up the first skystone
        possessSkyStone(trackingPose, overallTime);
        //  The heading needs to be used for the second skystone
        double skyStoneHeading = trackingPose.getTargetPose().getHeading();
        //  And figure out if the second skystone is against the wall, will require some rotation
        boolean extraSpinRequired = false;
        for (QuarryStone skyStone: QuarryStone.getSkyStones()){
            if (skyStone.getStoneLocation() == QuarryStone.StoneLocation.ZERO){
                extraSpinRequired = true;
            }
        }
        moveBackToClearQuarry(trackingPose, overallTime);
        //rotate to the foundation
        targetPose = new Pose (trackingPose.getX(), trackingPose.getY(), 0.0);
        trackingPose.setTargetPose(targetPose);
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);

        travelUnderBridgeToFoundation(trackingPose, overallTime);
        setLifterHeightToPlaceStone();  //raise lifter to place on foundation

        //**  Refine position to foundation
        refinePosition(FieldObject.FOUNDATION, trackingPose, overallTime);
        //  Dump stone
        autoReleaseStone(Speed.FAST, overallTime);
        if (debugOn) Log.d(logTag, "Stone released "  + overallTime.toString());

        //  Back away from Foundation Plate
        double headingOverride = (alliance == Alliance.RED) ? 15.0 : -15.0;
        targetPose = bridge.getFoundationSideStagingPose(Bridge.BridgeLane.INNER_LANE,headingOverride);
        trackingPose.setTargetPose(targetPose);
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "backed away from foundation plate "  + overallTime.toString());

        //drop lifter
        findLifterZero();


        //  Once arrive at the foundation plate, start extending tape measure
        StopWatch tapeExtendTime = new StopWatch();
        long tapeExtendTimeOut = 6000L;
        yoyo.setPower(YOYO_EXTEND);

        //  Wait for tape measure to stop
        while (opModeIsActive() && tapeExtendTime.getElapsedTimeMillis() < tapeExtendTimeOut){
            //Wait for the tape measure to extend
        }
        yoyo.setPower(YOYO_STOP);

    }

    protected TrackingPose surveilQuarry(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_surveyQuarry2";

        ArrayList<QuarryStone> observedQuarryStones = new ArrayList<>();
        assignObservedQuarryStones(observedQuarryStones,1);


        recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                , observedQuarryStones.get(2));
        if (debugOn) Log.d(logTag, "Quarry Observed " + overallTime.toString());

        //TrackingPose endPose = currentPose;
        if(QuarryStone.getCurrentCountSkyStones() == 0 && QuarryStone.getCountObservedStones() <= 1) {
            //If didn't see more than one stone, move right a little and try again
            if (debugOn) Log.d(logTag, "SkyStone not identified, extending search..." + QuarryStone.getCountObservedStones() +
                    " stone's observed");
            moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
            assignObservedQuarryStones(observedQuarryStones,2);

            recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                    , observedQuarryStones.get(2));
        }

        //  TODO:  Improve this logic for inclusion of second vantage point
        determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));
        return trackingPose;
    }

    protected void determineSkyStonePattern(QuarryStone firstStone, QuarryStone secondStone, StopWatch observationTimer){
        /**
         * Based on which stone is a SkyStone, generate the arraylist of stones
         */
        String logTag = "BTI_determineSkyStone.";
        Log.d(logTag, "Determining Stone Pattern...");
        QuarryStone.StoneLocation observedSkyStoneLocation;
        long currentTime = observationTimer.getElapsedTimeMillis();
        //Based on which position the observed skyStone was, the pa
        if (firstStone.isSkyStone(currentTime)){
            Log.d(logTag, "First stone is skystone " + firstStone.toString());
            observedSkyStoneLocation = firstStone.getStoneLocation();
        } else if (secondStone.isSkyStone(currentTime)){
            Log.d(logTag, "Second stone is skystone " + secondStone.toString());
            observedSkyStoneLocation = secondStone.getStoneLocation();
        } else {
            //Based on which position the observed skyStone was, the third is inferred
            Log.d(logTag, "Neither of the observed is skystone");
            Log.d(logTag, "Observed " + firstStone.toString() + " and " + secondStone.toString());
            observedSkyStoneLocation = inferThirdPosition(firstStone);
            Log.d(logTag, "Inferred SkyStone: " + observedSkyStoneLocation.toString());
        }

        //  Write the skyStones arraylist in QuarryStone static variable
        QuarryStone.setSkyStones(observedSkyStoneLocation);

        for (QuarryStone stone: QuarryStone.getQuarryStones()) {
            Log.d(logTag, "Writing stone data " + stone.toString());
        }

        for (QuarryStone skyStone: QuarryStone.getSkyStones()){
            Log.d(logTag, "SkyStone--> " + skyStone.toString());
        }

    }


    protected void surveilQuarryFromSecondPosition(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_surveyQuarry2";

        ArrayList<QuarryStone> observedQuarryStones = new ArrayList<>();
        //If didn't see more than one stone, move right a little and try again
        if (debugOn) Log.d(logTag, "SkyStone not identified, extending search..." + QuarryStone.getCountObservedStones() +
                " stone's observed");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        assignObservedQuarryStones(observedQuarryStones,2);

        recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                , observedQuarryStones.get(2));

        //  TODO:  Improve this logic for inclusion of second vantage point
        determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));
    }

    protected void recordQuarryObservationsDuringInit(QuarryStone firstStone, QuarryStone secondStone, QuarryStone thirdStone, StopWatch timer){

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions) {

                    // Determine which stone to update based on the position in the frame
                    QuarryStone currentStone;
                    if (alliance == Alliance.BLUE) {
                        if (recognition.getLeft() < 150) {
                            currentStone = thirdStone;
                        } else if (recognition.getLeft() > 350) {
                            currentStone = firstStone;
                        } else currentStone = secondStone;
                    } else{
                        if (recognition.getLeft() < 150) {
                            currentStone = firstStone;
                        } else if (recognition.getLeft() > 350) {
                            currentStone = thirdStone;
                        } else currentStone = secondStone;
                    }
                    //  See if it is recorded as skyStone
                    Boolean isSkyStone;
                    if (recognition.getLabel().equalsIgnoreCase("SkyStone")) {
                        isSkyStone = true;
                    } else isSkyStone = false;
                    //  Note, this recording of QuarryStone observations updates some of the
                    //  properties for the QuarryStone objects.
                    //  Notably:
                    //      *  It purges expired observations from the time stamped observation arrays
                    //      *  It populates the numObservations field based on the count of time-stamped entries
                    //      *  It populates the numIdSkyStone field based on how many of those remaining observations saw it as a SkyStone
                    //      These allow for simple calculation view isSkyStone() later
                    currentStone.recordTSObservation(timer.getElapsedTimeMillis(), isSkyStone);
                }
                if (updatedRecognitions.size() == 0){
                    //No stones were observed, purge out old observations
                    QuarryStone.purgeTSObservationsBefore(timer.getElapsedTimeMillis());
                }
            } else {
            }
        }
    }


    protected void possessSkyStone(TrackingPose trackingPose, StopWatch overallTime){
        //  grab a skystone
        boolean debugOn = true;
        String logTag = "BTI_possessSkyStone2";
        driveToSkyStone(trackingPose);
        if (debugOn) Log.d(logTag, "....COMPLETED Drive to Skystone " + overallTime.toString());
        if (debugOn) Log.d(logTag, "endPose Position:" + trackingPose.toString());

        if (debugOn) Log.d(logTag, "Refining position to stone...");
        refinePosition(FieldObject.QUARRY_STONE, trackingPose, overallTime);

        if (debugOn) Log.d(logTag, "...COMPLETED refining position to stone " + overallTime.toString());
        findLifterZeroWhileGrabbingStone();
    }

    private void driveToSkyStone(TrackingPose trackingPose){
        /**
         * Drive from current location to the next SkyStone
         */

        String logTag = "BTI_driveToSkyStone2";
        boolean debugOn = true;
        if (debugOn) Log.d(logTag, "starting driveToSkyStone, current Position " + trackingPose.toString());

        //Retrieve a skystone from those observed.  If not observed, select random
        QuarryStone skyStone = getTargetSkyStone();

        //Travel to the SkyStone
        if (debugOn) Log.d(logTag, "Setting target to skystone");
        setTargetPoseToSkyStone(skyStone, trackingPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving to skyStone at " + trackingPose.getTargetPose().toString());
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "SkyStone travel completed, Location: " + trackingPose.toString());

    }

    private void driveToSecondSkyStone(TrackingPose trackingPose){
        /**
         * Drive from current location to the next SkyStone
         */

        String logTag = "BTI_driveToSkyStone3";
        boolean debugOn = true;
        if (debugOn) Log.d(logTag, "starting driveToSecondSkyStone, current Position " + trackingPose.toString());

        //Retrieve a skystone from those observed.  If not observed, select random
        QuarryStone skyStone = getTargetSkyStone();

        //Travel to the SkyStone
        if (debugOn) Log.d(logTag, "Setting target to skystone");
        setTargetPoseToSkyStone(skyStone, trackingPose, 0.0);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving to skyStone at " + trackingPose.getTargetPose().toString());
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "SkyStone travel completed, Location: " + trackingPose.toString());
    }


    protected void moveBackToClearQuarry(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_moveBackToClearQ";

        if (debugOn) Log.d(logTag, "Moving back to clear Quarry...");

        //**  Move back a little to clear quarry
        double signYCoord = (alliance == Alliance.BLUE) ? 1.0 : -1.0;
        double backtrackDist = 10.0 * signYCoord;

        //First, make sure that the current position is updated
        EncoderTracker.updatePoseUsingThreeEncoders(trackingPose, imu);
        Pose targetPose = new Pose (trackingPose.getX(), trackingPose.getY() + backtrackDist, trackingPose.getHeading());
        trackingPose.setTargetPose(targetPose);
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "...COMPLETED MOVE BACK " + overallTime.toString());
        if (debugOn) Log.d(logTag, trackingPose.toString());

    }

    /*
    protected void travelUnderBridgeToFoundation(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_travelUnderBridge";

        //Travel under bridge toward foundation
        //But need to lift arm first, so to to  PoseAfterPlaceSkystone
        if (debugOn) Log.d(logTag, "Getting tracking pose across bridge ");
        setTargetPoseToDumpSkyStone(trackingPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge ");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "Travel across bridge completed, Location: " + trackingPose.toString());
        if (debugOn) Log.d(logTag, "overallTime: "  + overallTime.toString());
    }

     */

    protected void travelUnderBridgeToFoundation(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_travelUnderBridge";

        //Travel under bridge toward foundation
        //But need to lift arm first, so to to  PoseAfterPlaceSkystone
        //  Move to staging area
        if (debugOn) Log.d(logTag, "Getting tracking pose across bridge ");
        Pose targetPose = bridge.getQuarrySideStagingPose(Bridge.BridgeLane.INNER_LANE);
        trackingPose.setTargetPose(targetPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving to staging area bridge ");
        //Note, since the staging area is intended to be a pass-through point
        //The accuracy has been opened up to LOOSE, hopefully resulting in less slow-down by robot
        //TODO:  Verify that LOOSE Accuracy improves the slow-down situation
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, Accuracy.LOOSE, softStartConfig,imu, telemetry);


        //Move under the bridge
        if (debugOn) Log.d(logTag, "Getting tracking pose under bridge ");
        targetPose = bridge.getFoundationSideStagingPose(Bridge.BridgeLane.INNER_LANE) ;
        trackingPose.setTargetPose(targetPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving to staging area bridge ");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, Accuracy.LOOSE, softStartConfig,imu, telemetry);

        setLifterHeightToPlaceStone();

        //Move to foundation
        if (debugOn) Log.d(logTag, "Getting tracking pose to foundation ");
        setTargetPoseToDumpSkyStone(trackingPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving to staging area bridge ");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);


        if (debugOn) Log.d(logTag, "Travel across bridge completed, Location: " + trackingPose.toString());
        if (debugOn) Log.d(logTag, "overallTime: "  + overallTime.toString());
    }

    protected void backAwayFromFoundation(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_backAwayFromF";

        if (debugOn) Log.d(logTag, "Backing away from foundation plate");
        setTargetPoseAfterPlaceSkystone(trackingPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "Backed away, ready to drop lifter: " + trackingPose.toString());
        if (debugOn) Log.d(logTag, "overallTime: "  + overallTime.toString());
    }

    protected void parkUnderBridge(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_parkUnderBridge";
        setTargetPoseToBridgePark(trackingPose);
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~parking under bridge");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "...Parked under bridge " + trackingPose.toString());
        if (debugOn) Log.d(logTag, "Time " + overallTime.toString());

    }

    protected void setTargetPoseToBridgePark(TrackingPose trackingPose){
        String logTag = "BTI_getTracking.Park.";
        Log.d(logTag, "Creating tracking pose to park under bridge...");

        //Create the beginning and end poses for the move

        //Pose bridgePark = Pose.getBridgeParkPose(trackingPose, alliance);
        Pose bridgePark = bridge.getParkPose(Bridge.BridgeLane.INNER_LANE);
        Log.d(logTag, "Creating tracking pose to bridge park COMPLETED!");
        trackingPose.setTargetPose(bridgePark);
    }

    protected void setTargetPoseToSkyStone(QuarryStone skyStone, TrackingPose trackingPose){
        String logTag = "BTI_getTrack.SkyStone2.";
        double stoneX = skyStone.getX();   //retrieve x dimension
        //Determine if the offset should be added or subtracted based on alliance
        double offsetSign = (alliance == Alliance.BLUE) ? 1.0 : -1.0;
        double stoneY = skyStone.getY() + (offsetDistance * offsetSign);  //retrieve y dimension
        double stoneHeading;
        if (alliance == Alliance.RED){
            stoneHeading = 90.0;
        } else {
            stoneHeading = -90.0;
        }

        //Don't let the robot smash into the wall
        if (skyStone.getStoneLocation() == QuarryStone.StoneLocation.ZERO
            | skyStone.getStoneLocation() == QuarryStone.StoneLocation.ONE){
            stoneX = -55.0;
        }

        //Create the beginning and end poses for the move
        Pose skyStonePose = new Pose(stoneX,   stoneY , stoneHeading);
        trackingPose.setTargetPose(skyStonePose);
        Log.d(logTag, "Target pose replaced to " + skyStone.toString());
    }

    protected void setTargetPoseToSkyStone(QuarryStone skyStone, TrackingPose trackingPose, double prescribedHeading){
        String logTag = "BTI_getTrack.SkyStone3.";
        // First calculate the generic SkyStone pose
        setTargetPoseToSkyStone(skyStone, trackingPose);

        //  Then update the target heading to the prescribed angle
        Pose targetPose = trackingPose.getTargetPose();
        targetPose.setHeading(prescribedHeading);


        Log.d(logTag, "Target pose replaced to " + skyStone.toString());
    }

    protected void setTargetPoseToDumpSkyStone(TrackingPose trackingPose){
        String logTag = "BTI_getTracking.Bridge.";
        Log.d(logTag, "Creating tracking pose to en route to Foundation under bridge...");

        //Create the beginning and end poses for the move

        Pose acrossBridge = foundation.getSkyStoneDumpingPose(alliance);
        Log.d(logTag, "Creating tracking pose across bridge COMPLETED!");
        trackingPose.setTargetPose(acrossBridge);
    }


    protected void setTargetPoseAfterPlaceSkystone(TrackingPose trackingPose){
        String logTag = "BTI_getTracking.Place2.";
        Log.d(logTag, "Creating tracking pose after dropping skyStone...");

        //Create the beginning and end poses for the move

        //Note: this returns a relative position
        Pose afterPlaceSkyStone = foundation.getPoseAfterPlaceSkystone(trackingPose, alliance);
        Log.d(logTag, "Creating tracking pose after place SkyStone COMPLETED!");
        trackingPose.setTargetPose(afterPlaceSkyStone);
    }


    private void executeDelayedRoutine(TrackingPose trackingPose){
        String logTag = "BTI_executeDelayed";
        boolean debugOn = true;
        long waitTimeout = 20000L;
        long yoyoExtendTime = 5000L;
        long yoyoTimeout = waitTimeout + yoyoExtendTime;


        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Executing Delay");
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        //***************************************************************
        //  Wait for 20 seconds
        //***************************************************************
        telemetry.clear();
        while (opModeIsActive() && overallTime.getElapsedTimeMillis() < waitTimeout){
            telemetry.addData("Delaying, elapsed time", overallTime.toString());
            telemetry.update();
        }

        //***************************************************************
        //  EXTEND ARM AND START TAPE MOVING
        //***************************************************************
        if (debugOn) Log.d(logTag, "Extending arm");

        yoyo.setPower(YOYO_EXTEND);
        //Extend Arm, find zero, and set lifter to stone grab height
        raiseLifterToExtendArm();
        findLifterZero();
        if (debugOn) Log.d(logTag, "Arm extend complete " + overallTime.toString());


        //***************************************************************
        //  WAIT FOR THE TAPE MEASURE TO EXTEND
        //***************************************************************
        while(opModeIsActive() && overallTime.getElapsedTimeMillis() < yoyoTimeout){
            //Wait for the tape measure to extend
        }
        yoyo.setPower(YOYO_STOP);

        //***************************************************************
        //  MAKE ONLY MOVE
        //***************************************************************
        /*
        if (debugOn) Log.d(logTag, "Driving under Bridge");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "Drive complete, Overall Time: " + overallTime.toString());
         */
    }

    @Override
    protected TrackingPose refinePosition(FieldObject fieldObject, TrackingPose trackingPose, StopWatch overallTime) {
        double currentDistance = frontDistSensor.getDistance(DistanceUnit.INCH);
        double correctionSpeed = 0.15;
        double minRangeBlock = fieldObject.getMinDistance();
        double maxRangeBlock = fieldObject.getMaxDistance();
        boolean inRange = (currentDistance > minRangeBlock && currentDistance < maxRangeBlock) ? true : false;
        StopWatch timer = new StopWatch();
        long timeout = 2000L;
        boolean isTimedOut = false;

        boolean debugOn = true;
        String logTag = "BTI_refinePosition";
        if (debugOn) Log.d(logTag, "Refining position to " + fieldObject.name());

        while (opModeIsActive() && !inRange && !isTimedOut) {
            if (currentDistance > maxRangeBlock) {
                // move forward
                translateRobot(TranslateDirection.FORWARD, correctionSpeed);
            } else if (currentDistance < minRangeBlock) {
                // move backwards
                translateRobot(TranslateDirection.BACKWARD, correctionSpeed);
            }
            currentDistance = frontDistSensor.getDistance(DistanceUnit.INCH);
            inRange = (currentDistance > minRangeBlock && currentDistance < maxRangeBlock) ? true : false;
            isTimedOut = (timer.getElapsedTimeMillis() >= timeout) ? true : false;
            EncoderTracker.updatePoseUsingThreeEncoders(trackingPose, imu);

        }
        String resultString = (isTimedOut) ? "FAILURE - Timed out" : "SUCCESS";
        stopMotors();
        if (debugOn) Log.d(logTag, "COMPLETED Refining position to " + fieldObject.name() +
                " Result: " + resultString);
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        return trackingPose;
    }



    private void endInitTelemetry(){
        telemetry.clear();
        telemetry.addLine("Ready...");
        telemetry.addLine("Alliance: " + alliance.name());
        telemetry.addLine("Field Side: " + fieldSide.name());
        telemetry.addLine("Delayed Start: " + delayedStart.name());
        telemetry.update();

    }

    class ObserveQuarryDuringWait implements Runnable {
        volatile boolean keepRunning = true;

        //************************************************
        //  *******     CONSTRUCTORS       **************
        //************************************************
        public ObserveQuarryDuringWait(){}


        //************************************************
        //  *******     Class Methods       **************
        //************************************************
        public void run(){
            boolean debugOn = true;
            String logTag = "BTI_observeQuarryD.Wait";
            StopWatch observationTimer = new StopWatch();
            telemetry.clear();
            if (debugOn) Log.d(logTag, "Started concurrent thread");

            //Setup the array of observed quarry stones
            ArrayList<QuarryStone> observedQuarryStones = new ArrayList<>();
            assignObservedQuarryStones(observedQuarryStones,1);

            int i = 1;
            while (keepRunning) {

                if(debugOn) Log.d(logTag, "Entering loop, pass " + i);
                //Scan the quarry
                recordQuarryObservationsDuringInit(observedQuarryStones.get(0), observedQuarryStones.get(1)
                        , observedQuarryStones.get(2),observationTimer);

                telemetry.addData("Loop Count", i);
                telemetry.addData("Total Observations", QuarryStone.getTotalObservations());
                telemetry.addData("Stones Observed", QuarryStone.getCountObservedStones());
                telemetry.addData("SkyStones Observed", QuarryStone.getCurrentCountSkyStones());
                telemetry.addData("SkyStones written to Array", QuarryStone.getFoundSkyStoneCount());
                for(QuarryStone stone: QuarryStone.getSkyStones()){
                    telemetry.addData("SkyStone Position", stone.getStoneLocation().name());
                }
                telemetry.addLine("Ready...");
                telemetry.addLine("Alliance: " + alliance.name());
                telemetry.addLine("Field Side: " + fieldSide.name());
                telemetry.addLine("Delayed Start: " + delayedStart.name());

                telemetry.update();

                if(debugOn) Log.d(logTag, "loop: " + i + " totalObs: " + QuarryStone.getTotalObservations()
                        + " Stones Obs: " + QuarryStone.getCountObservedStones()
                        + " SkyStones Obs: " + QuarryStone.getCurrentCountSkyStones()
                        + " Written to Array: " + QuarryStone.getFoundSkyStoneCount());
                //If the skyStone has been observed, set the pattern
                //TODO: Must clear skystones written if currentCountSkyStones is zero

                //If the skyStone has been observed, set the pattern
                if (QuarryStone.getCurrentCountSkyStones() == 1) {
                    //  if the stones aren't already written
                    if(debugOn) Log.d(logTag, "Stone observed, may write pattern");

                    boolean observedSkyStoneAlreadyFound = false;
                    for(QuarryStone stone: observedQuarryStones){
                        if(stone.isSkyStone()) {
                            observedSkyStoneAlreadyFound = stone.isSkyStoneAlreadyInSkyStonesArray();
                            if(debugOn) Log.d(logTag, "Skystone in position " + stone.getStoneLocation().name() +
                                    " new info? " + observedSkyStoneAlreadyFound);
                        }
                    }

                    if (QuarryStone.getFoundSkyStoneCount() == 0 | !observedSkyStoneAlreadyFound) {
                        if(debugOn) Log.d(logTag, "Pattern Recorded");
                        determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));
                    } else {
                        if(debugOn) Log.d(logTag, "Info is not new, no write");
                    }

                } else if (QuarryStone.getCountObservedStones() == 2){
                    //  If 2 non-skyStones have been observed, there is enough info to set pattern
                    determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));
                } else {
                    //  Otherwise, not enough info to determine skystone, clear the skystone array
                    QuarryStone.getSkyStones().clear();
                }

                try{
                    Thread.sleep(1000);
                    i++;
                } catch (InterruptedException e) {
                    if (debugOn) Log.d(logTag, "Interruption Exception encountered");
                    //todo: verify this runs before main thread processes skystones
                    //determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));

                }
            }
            if (debugOn) Log.d(logTag, "observation loop exited");
        }
    }
}
