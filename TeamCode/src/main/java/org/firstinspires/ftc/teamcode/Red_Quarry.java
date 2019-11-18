package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

import static java.lang.String.format;

@Autonomous
public class Red_Quarry extends eBotsAuton2019 {


    /****************************************************************
     //******    CONFIGURATION PARAMETERS
     //***************************************************************/
    private Alliance alliance = Alliance.RED;
    private FieldSide fieldSide = FieldSide.QUARRY;
    private Speed speedConfig = Speed.SLOW;
    private GyroSetting gyroConfig = GyroSetting.EVERY_LOOP;
    private SoftStart softStartConfig = SoftStart.MEDIUM;
    private Accuracy accuracyConfig = Accuracy.STANDARD;


    @Override
    public void runOpMode(){
        /** APPLY CONFIG     **************/
        setGyroConfiguration(gyroConfig);      //Set gyro parameters and initialize
        setSpeedConfiguration(speedConfig);    //Set speed and PID gain limits
        setSoftStartConfig(softStartConfig);    //Set the soft start settings
        setAccuracyLimits(accuracyConfig);      //Set the accuracy limits
        setAllianceObjects(alliance);           //Create objects for Quarry Stones and Foundation
        simulateMotors = false;
        String logTag = "BTI_AutonCameraBeta";

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


        //  *********  INITIALIZE FOR FIRST MOVEMENT   *****************
        //  Create currentPose, which is the tracked position from start to target
        //  This is a special type of pose that is intended to track the path of travel
        //  It has a targetPose, which is the intended destination
        //  It also has an error object which tracks it's status relative to targetPose

        //Initialize Poses
        Pose startPose;
        if (fieldSide == FieldSide.QUARRY){
            startPose = new Pose(Pose.StartingPose.QUARRY);
        } else {
            startPose = new Pose(Pose.StartingPose.FOUNDATION);
        }

        if (alliance == Alliance.RED){
            applyRedAlliancePoseTransform(startPose);
        }

        //  This target pose is used if SkyStone not found in first scan
        Pose targetPose = new Pose(startPose.getX()-6.5, startPose.getY(), startPose.getHeading());
        TrackingPose currentPose = new TrackingPose(startPose, targetPose);
        TrackingPose endPose = currentPose;
        //  Capture the initial gyro offset for later use, it must be passed to each tracking pose
        if (useGyroForNavigation) {
            currentPose.setInitialGyroOffset(getGyroReadingDegrees());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }

        telemetry.clear();
        telemetry.addLine("Ready...");
        telemetry.update();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************

        waitForStart();

        StopWatch overallTime = new StopWatch();

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************
        //Extend Arm
        raiseLifterToExtendArm();
        Log.d(logTag, "Arm Extended " + overallTime.toString());
        findLifterZero();
        Log.d(logTag, "Arm Zeroed " + overallTime.toString());
        setLifterHeightToGrabStone();
        Log.d(logTag, "Ready to grab stone " + overallTime.toString());

        ArrayList<QuarryStone> observedQuarryStones = new ArrayList<>();
        assignObservedQuarryStones(alliance, observedQuarryStones,1);


        recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                , observedQuarryStones.get(2), alliance);
        Log.d(logTag, "Quarry Observed " + overallTime.toString());

        if(QuarryStone.getCountSkyStones() == 0 && QuarryStone.getCountObserved() <= 1) {
            //If didn't see more than one stone, move right a little and try again
            Log.d(logTag, "SkyStone not identified, extending search..." + QuarryStone.getCountObserved() +
                    " stone's observed");
            endPose = travelToNextPose(currentPose, motorList);
            assignObservedQuarryStones(alliance, observedQuarryStones,1);

            recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                    , observedQuarryStones.get(2), alliance);
        }

        //  TODO:  Improve this logic for inclusion of second vantage point
        determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));


        //  grab a skystone
        endPose = driveToSkyStone(endPose, motorList, alliance);
        Log.d(logTag, "....COMPLETED Drive to Skystone " + overallTime.toString());
        Log.d(logTag, "endPose Position:" + endPose.toString());

        Log.d(logTag, "Refining position to stone...");
        refinePosition(FieldObject.QUARRY_STONE);
        Log.d(logTag, "...COMPLETED refining position to stone " + overallTime.toString());
        autoGrabBlockNoMovement(getDriveMotors());

        Log.d(logTag, "Moving back to clear Quarry...");

        //**  Move back a little to clear quarry
        double signYCoord = (alliance == Alliance.BLUE) ? 1.0 : -1.0;
        double backtrackDist = 10.0 * signYCoord;
        startPose = new Pose (endPose.getX(), endPose.getY(), endPose.getHeading());
        targetPose = new Pose (endPose.getX(), endPose.getY() + backtrackDist, endPose.getHeading());
        currentPose = new TrackingPose(startPose, targetPose, endPose.getInitialGyroOffset());
        endPose = travelToNextPose(currentPose, motorList);
        Log.d(logTag, "...COMPLETED MOVE BACK");
        Log.d(logTag, endPose.toString());


        //Travel under bridge to foundation
        Log.d(logTag, "Getting tracking pose across bridge");
        currentPose = getTrackingPoseAcrossBridge(endPose);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge");
        endPose = travelToNextPose(currentPose, motorList);     //Actually drive
        Log.d(logTag, "Travel across bridge completed, Location: " + endPose.toString());
        Log.d(logTag, "overallTime: "  + overallTime.toString());

        //Now spin toward Foundation to dump stone
        setLifterHeightToPlaceStone();  //raise lifter to place on foundation

        //**  Correct Heading
        correctHeading(endPose, motorList);

        //Now try to refine position
        /*
        //  Move forward a little bit more to dump block on foundation
        Log.d(logTag, "Getting tracking pose to foundation");
        currentPose = getTrackingPoseToFoundation(endPose);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving to foundation");
        endPose = travelToNextPose(currentPose, motorList);     //Actually drive
        Log.d(logTag, "Foundation travel completed, Location: " + currentPose.toString());
        Log.d(logTag, "overallTime: "  + overallTime.toString());

        //**  Correct Heading
        correctHeading(endPose, motorList);
        */
        Log.d(logTag, "Refining position to foundation...");
        refinePosition(FieldObject.FOUNDATION);
        Log.d(logTag, "...COMPLETED Refining position to foundation " + overallTime.toString());


        //  Dump stone
        autoReleaseStone(motorList);
        Log.d(logTag, "Stone released "  + overallTime.toString());


        //  move back a little to clear foundation plate
        //refinePosition(20.0, 25.0);

        Log.d(logTag, "Backing away from foundation plate");
        currentPose = getTrackingPoseAfterPlaceSkystone(endPose, alliance);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge");
        endPose = travelToNextPose(currentPose, motorList);     //Actually drive
        Log.d(logTag, "Backed away, ready to drop lifter: " + currentPose.toString());
        Log.d(logTag, "overallTime: "  + overallTime.toString());


        //drop lifter
        findLifterZero();

        //  Now park under the bridge
        currentPose = getTrackingPoseToBridgePark(currentPose, alliance);
        Log.d(logTag, "~~~~~~~~~~~~~parking under bridge");
        endPose = travelToNextPose(currentPose, motorList);     //Actually drive
        Log.d(logTag, "...Parked under bridge " + endPose.toString());
        Log.d(logTag, "Time " + overallTime.toString());


        //  Verify all drive motors are stopped
        if(!simulateMotors) stopMotors(motorList);

        Log.d(logTag, "Closing camera");
        //  Close vuforia webcam interface
        if (tfod != null) {
            tfod.shutdown();
        }

    }

    protected void writeOdometryTelemetry(String loopMetrics, TrackingPose currentPose) {
        telemetry.addData("Loop Metrics: ", loopMetrics);

        telemetry.addData("currentPose: ", currentPose.toString());
        telemetry.addData("targetPose: ", currentPose.getTargetPose().toString());
        telemetry.addData("currentError: ", currentPose.printError());

        telemetry.addData("Loop Error Condition Check: ", format("%.3f", currentPose.getErrorControlValue()));
        telemetry.addLine(printDriveSignalMetrics());

        telemetry.addData("Heading Locked: ", currentPose.isHeadingErrorLocked());
        telemetry.addData("forwardTracker: ", forwardTracker.toString());
        telemetry.addData("lateralTracker: ", lateralTracker.toString());
        telemetry.update();
    }

    private void writeFinalTelemetry(StopWatch overallTime){
        String logTag = "BTI_writeFinalTel.." ;
        Log.d(logTag, "Writing final telemetry");

        //Stop and hold telemetry for a while
        Log.d(logTag, "Total time: " + overallTime.toString());
        Log.d(logTag, "Total Stones in quarry: " + QuarryStone.getQuarryStones().size());
        Log.d(logTag, "Skystones found: " + QuarryStone.getFoundSkyStoneCount());
        telemetry.clear();
        telemetry.addData("Overall Time", overallTime.toString());
        telemetry.addData("SkyStones Found", QuarryStone.getFoundSkyStoneCount());
        for (QuarryStone stone: QuarryStone.getQuarryStones()) {
            Log.d(logTag, "Writing stone data " + stone.toString());
            telemetry.addLine(stone.toString());
        }
        telemetry.update();

    }

}
