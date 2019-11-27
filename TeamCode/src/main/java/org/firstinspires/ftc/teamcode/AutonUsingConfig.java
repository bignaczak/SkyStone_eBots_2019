package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.ListIterator;

@Autonomous
public class AutonUsingConfig extends eBotsAuton2019 {

    /****************************************************************
     //******    CONFIGURATION PARAMETERS
     //***************************************************************/
    private Speed speedConfig = Speed.FAST;
    private GyroSetting gyroConfig = GyroSetting.EVERY_LOOP;
    private SoftStart softStartConfig = SoftStart.MEDIUM;
    private Accuracy accuracyConfig = Accuracy.STANDARD;

    //  These get assigned through the configuration file
    private Alliance alliance;
    private FieldSide fieldSide;
    private DelayedStart delayedStart;
    private StopWatch overallTime;

    boolean debugOn = true;
    @Override
    public void runOpMode(){
        boolean debugOn = true;
        String logTag = "BTI_AutonUsingConfig";
        if (debugOn) Log.d(logTag, "Entering AutonUsingConfig");

        //Apply the configurations
        importConfigurationFile();

        /** APPLY CONFIG     **************/
        setGyroConfiguration(gyroConfig);      //Set gyro parameters and initialize
        setSpeedConfiguration(speedConfig);    //Set speed and PID gain limits
        setSoftStartConfig(softStartConfig);    //Set the soft start settings
        setAccuracyLimits(accuracyConfig);      //Set the accuracy limits
        setAllianceObjects(alliance);           //Create objects for Quarry Stones and Foundation
        simulateMotors = false;

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
            initializeManipMotors();            //manip motors
            initializeLimitSwitches();          //limit switches
            initializeDistanceSensors();        //distance sensors
        }


        //***************************************************************
        //  Open up the webcam
        //***************************************************************
        prepWebcam();

        //  Create the first TrackingPose
        //  Note, setting the initialGyroOffset is critical
        TrackingPose currentPose = getFirstTrackingPose();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        endInitTelemetry();
        waitForStart();

        overallTime = new StopWatch();

        if (delayedStart == DelayedStart.YES){
            executeDelayedRoutine(currentPose);
        }else if(fieldSide == FieldSide.QUARRY){
            executeQuarryRoutine(currentPose);
        } else {
            executeFoundationRoutine(currentPose);
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

    private Pose getOpModeStartPose() {
        Pose startPose;
        if (fieldSide == FieldSide.QUARRY){
            startPose = new Pose(Pose.StartingPose.QUARRY);
        } else {
            startPose = new Pose(Pose.StartingPose.FOUNDATION);
        }

        if (alliance == Alliance.RED){
            applyRedAlliancePoseTransform(startPose);
        }
        return startPose;
    }

    protected void importConfigurationFile(){
        //Read in the configuration file
        String logTag = "BTI_importConfigFile";
        Log.d(logTag, "Entering importConfigurationFile");

        //Read in the configuration settings as an arraylist
        ArrayList<String> configArray = eBotsConfigIO.parseAutonConfigFile();

        //Convert the read-in text values into enumerations
        ListIterator<String> listIterator = configArray.listIterator();

        //Get all the config values, starting with alliance
        String configValue = listIterator.next();

        for(Alliance a: Alliance.values()){
            if (configValue.equalsIgnoreCase(a.name())){
                alliance = a;
                Log.d(logTag, "Alliance Assigned: " + alliance.name());
            }
        }

        //Next config is field side
        configValue = listIterator.next();

        for(FieldSide f: FieldSide.values()){
            if (configValue.equalsIgnoreCase(f.name())){
                fieldSide = f;
                Log.d(logTag, "Field Side Assigned: " + fieldSide.name());
            }
        }

        //Next config is delayed start
        configValue = listIterator.next();

        for(DelayedStart d: DelayedStart.values()){
            if (configValue.equalsIgnoreCase(d.name())){
                delayedStart = d;
                Log.d(logTag, "Delayed Start Assigned: " + delayedStart.name());
            }
        }
    }

    private TrackingPose getFirstTrackingPose(){
        //  *********  INITIALIZE FOR FIRST MOVEMENT   *****************
        //  Create currentPose, which is the tracked position from start to target
        //  This is a special type of pose that is intended to track the path of travel
        //  It has a targetPose, which is the intended destination
        //  It also has an error object which tracks it's status relative to targetPose
        boolean debugOn = true;
        String logTag = "BTI_getFirstTrack";
        if (debugOn) Log.d(logTag, "Entering getFirstTrackingPose " + alliance.name() + " " + fieldSide.name() + " " + delayedStart.name());

        Pose startPose;
        Pose targetPose;
        //  The first target pose depends on which side is being pursued
        if (fieldSide == FieldSide.QUARRY){
            //  This target pose is used if SkyStone not found in first scan
            startPose = getOpModeStartPose();
            targetPose = new Pose(startPose.getX()-6.5, startPose.getY(), startPose.getHeading());
        } else {
            //  Delayed Start and Foundation side uses a series of poses
            wayPoses = new ArrayList<>();
            setWayPoses(wayPoses, alliance, fieldSide, delayedStart);
            ListIterator<Pose> listIterator = wayPoses.listIterator();
            if (debugOn) Log.d(logTag, "wayPose count: " + wayPoses.size());

            if (listIterator.hasNext()) {
                //Make sure have enough poses to construct StartPose
                startPose = listIterator.next();
                //Now remove the first item from the list
                listIterator.remove();
                if (debugOn) Log.d(logTag, "startPose defined, wayPose count: " + wayPoses.size());

            } else{
                startPose = getOpModeStartPose();
            }

            if (listIterator.hasNext()) {  //Make sure there are at least 2 points
                targetPose = listIterator.next();
                //Now remove the first item from the list
                listIterator.remove();
                if (debugOn) Log.d(logTag, "targetPose defined, wayPose count: " + wayPoses.size());

            } else {    //Set end point same as start point (road to nowhere)
                targetPose = startPose;
            }
        }
        TrackingPose currentPose = new TrackingPose(startPose, targetPose);

        //  IMPORTANT STEP
        //  Capture the initial gyro offset for later use, it must be passed to each tracking pose
        if (useGyroForNavigation) {
            currentPose.setInitialGyroOffset(getGyroReadingDegrees());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }
        if (debugOn) Log.d(logTag, "Tracking Pose acquired " + currentPose.toString());
        if (debugOn) Log.d(logTag, "with targetPose acquired " + currentPose.getTargetPose().toString());
        return currentPose;
    }

    private void executeQuarryRoutine(TrackingPose currentPose){
        String logTag = "BTI_executeQuarry";
        if (debugOn) Log.d(logTag, "Starting executeQuarryRoutine...");

        TrackingPose endPose = currentPose;

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************
        //Extend Arm, find zero, and set lifter to stone grab height
        raiseLifterToExtendArm();
        findLifterZero();
        setLifterHeightToGrabStone();
        if (debugOn) Log.d(logTag, "Ready to grab stone " + overallTime.toString());


        //  get first SkyStone to foundation
        endPose = surveilQuarry(alliance, currentPose, overallTime);
        endPose = possessSkyStone(endPose, alliance, overallTime);
        endPose = moveBackToClearQuarry(endPose, alliance, overallTime);
        endPose = travelUnderBridgeToFoundation(endPose, alliance, overallTime);
        setLifterHeightToPlaceStone();  //raise lifter to place on foundation

        //**  Correct Heading
        correctHeading(endPose);
        //**  Refine position to foundation
        endPose = refinePosition(FieldObject.FOUNDATION, endPose, overallTime);
        //  Dump stone
        autoReleaseStone(Speed.FAST, overallTime);
        if (debugOn) Log.d(logTag, "Stone released "  + overallTime.toString());

        //  Back away from Foundation Plate
        endPose = backAwayFromFoundation(endPose, alliance, overallTime);

        //drop lifter
        findLifterZero();

        //Go get the other SkyStone
        //  Can't use possessSkyStone for this because lifter is down and must spin before can grab
        endPose = driveToSkyStone(endPose, alliance);
        if (debugOn) Log.d(logTag, "....COMPLETED Drive to second Skystone " + overallTime.toString());
        if (debugOn) Log.d(logTag, "endPose Position:" + endPose.toString());

        setLifterHeightToGrabStone();
        if (debugOn) Log.d(logTag, "Ready to grab stone " + overallTime.toString());

        //**  Correct Heading -- This will spin robot to pick up stone
        correctHeading(endPose);

        endPose = refinePosition(FieldObject.QUARRY_STONE, endPose, overallTime);
        autoGrabBlockNoMovement(getDriveMotors());
        //**  Move back a little to clear quarry
        endPose = moveBackToClearQuarry(endPose, alliance, overallTime);
        //Travel under bridge toward foundation
        endPose = travelUnderBridgeToFoundation(endPose, alliance, overallTime);
        setLifterHeightToPlaceStone();  //raise lifter to place on foundation

        //**  Correct Heading
        correctHeading(endPose);

        //**  Refine position to foundation
        if (debugOn) Log.d(logTag, "Refining position to foundation...");
        endPose = refinePosition(FieldObject.FOUNDATION, endPose, overallTime);
        if (debugOn) Log.d(logTag, "...COMPLETED Refining position to foundation " + overallTime.toString());


        //  Dump stone
        autoReleaseStone(Speed.FAST, overallTime);
        if (debugOn) Log.d(logTag, "Second skyStone released "  + overallTime.toString());
        //  Back away from Foundation Plate
        endPose = backAwayFromFoundation(endPose, alliance, overallTime);

        //drop lifter
        findLifterZero();

        //  Now park under the bridge
        endPose = parkUnderBridge(endPose, alliance, overallTime);

        //Now lower rake to ensure is parked under bridge
        lowerRake();
    }

    private void executeFoundationRoutine(TrackingPose currentPose){
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

        TrackingPose endPose = travelToNextPose(currentPose);
        if (debugOn) Log.d(logTag, "Leg " + travelLegCount + " completed, new position " + endPose.toString());
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        travelLegCount++;

        while(opModeIsActive() && listIterator.hasNext()) {

            //Correct heading angle if not within tolerance limits set in accuracyConfig
            correctHeading(currentPose);
            //Perform actions specified in the targetPose of the path leg that was just completed
            executePostMoveActivity(currentPose, alliance);

            //Set ending of previous leg as the starting pose for new leg
            //  Position and pose are taken from the TrackingPose object returned by travelToNextPose
            //  TODO:  This is not necessary, should just replace the targetPose of the existing trackingPose
            //   instead, eliminating need to copy initialGyroOffset

            Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
            Pose targetPose = listIterator.next();
            currentPose = new TrackingPose(startPose, targetPose);
            //Remove the item from the wayPose list
            listIterator.remove();
            //Must set the initialGryOffset
            currentPose.copyInitialGyroOffsetBetweenLegs(endPose.getInitialGyroOffset());

            if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Starting Leg " + travelLegCount.toString()
                    + " to " + currentPose.getTargetPose().toString());
            if (debugOn) Log.d(logTag, "Start Position " + currentPose.toString());
            endPose = travelToNextPose(currentPose);
            if (debugOn) Log.d(logTag, "Leg " + travelLegCount + " completed, new position " + endPose.toString());
            if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

            travelLegCount++;
        }
        if (debugOn) Log.d(logTag, "All Legs Completed, current Position " + endPose.toString());
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        correctHeading(currentPose);
    }

    private void executeDelayedRoutine(TrackingPose currentPose){
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
        TrackingPose endPose = travelToNextPose(currentPose);
        if (debugOn) Log.d(logTag, "Drive complete, Overall Time: " + overallTime.toString());

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
