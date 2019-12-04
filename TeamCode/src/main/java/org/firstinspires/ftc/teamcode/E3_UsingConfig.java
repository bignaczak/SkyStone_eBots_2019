package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.ListIterator;

import static org.firstinspires.ftc.teamcode.eBotsMotionController.moveToTargetPose;

@Autonomous
public class E3_UsingConfig extends eBotsAuton2019 {



    @Override
    public void runOpMode(){
        boolean debugOn = true;
        String logTag = "BTI_E3_UsingConfig";

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
        simulateMotors = false;

        Log.d(logTag, "Starting OpMode...");

        //  Setup motors & encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        } else{
            initializeDriveMotors(true);
            initializeEncoderTrackers(getDriveMotors());
            initializeManipMotors();
            initializeLimitSwitches();          //limit switches
            initializeDistanceSensors();        //distance sensors

        }

        if (debugOn) Log.d(logTag, "Encoders initialized " + EncoderTracker.getEncoderTrackerCount() + " found");

        //***************************************************************
        //  Open up the webcam
        //***************************************************************
        prepWebcam();

        //  Create the first TrackingPose
        //  Note, setting the initialGyroOffset is critical
        TrackingPose trackingPose = getFirstTrackingPose();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        endInitTelemetry();
        waitForStart();

        overallTime = new StopWatch();


        //**************************************************************
        //  ****        END OF INIT, WAIT FOR START         ************
        //**************************************************************
        telemetry.clear();
        telemetry.addLine("Ready...");
        telemetry.update();
        waitForStart();

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
        if (tfod != null) {
            tfod.shutdown();
        }
        if (debugOn) Log.d(logTag, "OpMode Complete, Overall Time: " + overallTime.toString());
    }
    private void executeFoundationRoutine(TrackingPose trackingPose){
        String logTag = "BTI_executeFound";
        boolean debugOn = true;

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
        if (debugOn) Log.d(logTag, "All Legs Completed, current Position " + trackingPose.toString());
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());
    }



    private void executeQuarryRoutine(TrackingPose trackingPose){
        boolean debugOn = true;
        String logTag = "BTI_executeQuarry";
        if (debugOn) Log.d(logTag, "Starting executeQuarryRoutine...");

        //TrackingPose endPose = trackingPose;

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************
        //Extend Arm, find zero, and set lifter to stone grab height
        raiseLifterToExtendArm();
        findLifterZero();
        setLifterHeightToGrabStone();
        if (debugOn) Log.d(logTag, "Ready to grab stone " + overallTime.toString());


        //  get first SkyStone to foundation
        surveilQuarry(alliance, trackingPose, overallTime);
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
        travelUnderBridgeToFoundation(trackingPose, overallTime);
        setLifterHeightToPlaceStone();  //raise lifter to place on foundation

        //**  Refine position to foundation
        refinePosition(FieldObject.FOUNDATION, trackingPose, overallTime);
        //  Dump stone
        autoReleaseStone(Speed.FAST, overallTime);
        if (debugOn) Log.d(logTag, "Stone released "  + overallTime.toString());

        //  Back away from Foundation Plate
        backAwayFromFoundation(trackingPose, overallTime);

        //drop lifter
        findLifterZero();

        //Go get the other SkyStone
        //  Can't use possessSkyStone for this because lifter is down and must spin before can grab

        driveToSecondSkyStone(trackingPose);        //Need new method to prevent spinning
        if (debugOn) Log.d(logTag, "....COMPLETED Drive to second Skystone " + overallTime.toString());
        if (debugOn) Log.d(logTag, "endPose Position:" + trackingPose.toString());

        setLifterHeightToGrabStone();

        //  Now spin
        trackingPose.getTargetPose().setHeading(skyStoneHeading);
        //  Apply extra spin if skystone is against wall
        if (extraSpinRequired) {
            Pose targetPose = trackingPose.getTargetPose();
            double newAngle = (Math.abs(skyStoneHeading) + 15) * Math.signum(skyStoneHeading);
            targetPose.setHeading(newAngle);
        }
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);

        if (debugOn) Log.d(logTag, "Ready to grab stone " + overallTime.toString());

        refinePosition(FieldObject.QUARRY_STONE, trackingPose, overallTime);
        autoGrabBlockNoMovement(getDriveMotors());
        //**  Move back a little to clear quarry
        moveBackToClearQuarry(trackingPose, overallTime);
        //Travel under bridge toward foundation
        travelUnderBridgeToFoundation(trackingPose, overallTime);
        setLifterHeightToPlaceStone();  //raise lifter to place on foundation

        //**  Refine position to foundation
        if (debugOn) Log.d(logTag, "Refining position to foundation...");
        refinePosition(FieldObject.FOUNDATION, trackingPose, overallTime);
        if (debugOn) Log.d(logTag, "...COMPLETED Refining position to foundation " + overallTime.toString());


        //  Dump stone
        autoReleaseStone(Speed.FAST, overallTime);
        if (debugOn) Log.d(logTag, "Second skyStone released "  + overallTime.toString());
        //  Back away from Foundation Plate
        backAwayFromFoundation(trackingPose, overallTime);

        //drop lifter
        findLifterZero();

        //  Now park under the bridge
        parkUnderBridge(trackingPose, overallTime);

        //Now lower rake to ensure is parked under bridge
        lowerRake();
    }

    @Override
    protected TrackingPose surveilQuarry(Alliance alliance, TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_surveyQuarry2";

        ArrayList<QuarryStone> observedQuarryStones = new ArrayList<>();
        assignObservedQuarryStones(alliance, observedQuarryStones,1);


        recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                , observedQuarryStones.get(2), alliance);
        if (debugOn) Log.d(logTag, "Quarry Observed " + overallTime.toString());

        //TrackingPose endPose = currentPose;
        if(QuarryStone.getCountSkyStones() == 0 && QuarryStone.getCountObserved() <= 1) {
            //If didn't see more than one stone, move right a little and try again
            if (debugOn) Log.d(logTag, "SkyStone not identified, extending search..." + QuarryStone.getCountObserved() +
                    " stone's observed");
            moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
            assignObservedQuarryStones(alliance, observedQuarryStones,1);

            recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                    , observedQuarryStones.get(2), alliance);
        }

        //  TODO:  Improve this logic for inclusion of second vantage point
        determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));
        return trackingPose;
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
        autoGrabBlockNoMovement(getDriveMotors());
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

        //Modified in V2 to execute the turn prior to driving to quarry
        Pose targetPose = new Pose (trackingPose.getX(), trackingPose.getY() + backtrackDist, 0.0);
        trackingPose.setTargetPose(targetPose);
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "...COMPLETED MOVE BACK " + overallTime.toString());
        if (debugOn) Log.d(logTag, trackingPose.toString());

    }

    protected void travelUnderBridgeToFoundation(TrackingPose trackingPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_travelUnderBridge";

        //Travel under bridge toward foundation
        //But need to lift arm first, so to to  PoseAfterPlaceSkystone
        if (debugOn) Log.d(logTag, "Getting tracking pose across bridge ");
        setTargetPoseAcrossBridge(trackingPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge ");
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

        Pose bridgePark = Pose.getBridgeParkPose(trackingPose, alliance);
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


    protected void setTargetPoseAcrossBridge(TrackingPose trackingPose){
        String logTag = "BTI_getTracking.Bridge.";
        Log.d(logTag, "Creating tracking pose to en route to Foundation under bridge...");

        //Create the beginning and end poses for the move

        Pose acrossBridge = foundation.getSkyStoneDumpingPose();
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
        long timeout = 20000L;

        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Executing Delay");
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        //***************************************************************
        //  Wait for 20 seconds
        //***************************************************************
        telemetry.clear();
        while (overallTime.getElapsedTimeMillis() < timeout){
            telemetry.addData("Delaying, elapsed time", overallTime.toString());
            telemetry.update();
        }

        //***************************************************************
        //  EXTEND ARM
        //***************************************************************
        if (debugOn) Log.d(logTag, "Extending arm");
        //Extend Arm, find zero, and set lifter to stone grab height
        raiseLifterToExtendArm();
        findLifterZero();
        if (debugOn) Log.d(logTag, "Arm extend complete " + overallTime.toString());

        //***************************************************************
        //  MAKE ONLY MOVE
        //***************************************************************

        if (debugOn) Log.d(logTag, "Driving under Bridge");
        moveToTargetPose(trackingPose, speedConfig, gyroConfig, accuracyConfig, softStartConfig,imu, telemetry);
        if (debugOn) Log.d(logTag, "Drive complete, Overall Time: " + overallTime.toString());
    }

    @Override
    protected TrackingPose refinePosition(FieldObject fieldObject, TrackingPose trackingPose, StopWatch overallTime) {
        double currentDistance = frontDistSensor.getDistance(DistanceUnit.INCH);
        double correctionSpeed = 0.15;
        double minRangeBlock = fieldObject.getMinDistance();
        double maxRangeBlock = fieldObject.getMaxDistance();
        boolean inRange = (currentDistance > minRangeBlock && currentDistance < maxRangeBlock) ? true : false;
        StopWatch timer = new StopWatch();
        long timeout = 1500L;
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

        if (debugOn) Log.d(logTag, "COMPLETED Refining position to " + fieldObject.name() +
                " Result: " + resultString);
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        return trackingPose;
    }

    @Override
    public void waitForStart(){
        super.waitForStart();
    }



    private void endInitTelemetry(){
        telemetry.clear();
        telemetry.addLine("Ready...");
        telemetry.addLine("Alliance: " + alliance.name());
        telemetry.addLine("Field Side: " + fieldSide.name());
        telemetry.addLine("Delayed Start: " + delayedStart.name());
        telemetry.update();

    }

}
