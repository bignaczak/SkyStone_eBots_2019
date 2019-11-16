package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.ListIterator;

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
            initializeManipMotors();
            initializeLimitSwitches();
        }

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
        //TODO:  Eliminate this variable, pass the torch between tracking poses
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

        //***************************************************************
        //  MAKE FIRST MOVE
        //***************************************************************
        //TODO: See if endPose can be eliminated and just keep using currentPose (I think endPose is a pointer to currentPose)
        TrackingPose endPose = travelToNextPose(currentPose, motorList);
        Log.d(logTag, "completed first leg " + overallTime.toString());

        //Correct heading angle if not within tolerance limits set in accuracyConfig
        Log.d(logTag, "correcting angle...");
        correctHeading(currentPose, motorList);

        //Perform actions specified in the targetPose of the path leg that was just completed
        executePostMoveActivity(currentPose, motorList, alliance);
        Log.d(logTag, "completed post move activity for first leg");
        int legCount = 1;
        Double currentPoseHeadingError;
        while(opModeIsActive() && iterator.hasNext()) {
            Log.d(logTag, "~~~~~~~~~~~Beginning of Loop for poses~~~~~~~~~~~~");

            //Correct heading angle if not within tolerance limits set in accuracyConfig
            Log.d(logTag, "correcting angle...");
            correctHeading(currentPose, motorList);

            //Set ending of previous leg as the starting pose for new leg
            //  Position and pose are taken from the TrackingPose object returned by travelToNextPose
            //  Note: Using the actual position and not the target position from targetPose
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
            //TODO:  Replace initialGyroOffset with a call to endPose.getInitialGyroOffset()
            Log.d(logTag, "previousInitialGyroOffset: " + initialGyroOffset);
            //  Note: pass the initial gyro offset from initialization to next tracking pose
            currentPose = new TrackingPose(startPose, nextPose, initialGyroOffset);
            Log.d(logTag, "...tracking pose created");

            Log.d(logTag, "~~~~~~~~~~~~~Starting Leg " + ++legCount);
            endPose = travelToNextPose(currentPose, motorList);
            Log.d(logTag, "Completed leg " + legCount + " : " + overallTime.toString());

            correctHeading(currentPose, motorList);
            //Perform actions specified in the targetPose of the path leg that was just completed
            executePostMoveActivity(currentPose, motorList, alliance);
            Log.d(logTag, "completed post move activity " + overallTime.toString());

            Log.d(logTag, "~~~~~~~~~~~End of Loop for poses~~~~~~~~~~~~");
        }

        Log.d(logTag, "Final pose reached, Correcting heading...");


        currentPose = deliverSkyStonesToBuildingZone(endPose, motorList, alliance);
        Log.d(logTag, "first stone delivered " + overallTime.toString());

        correctHeading(currentPose, motorList);

        //  Now park under the bridge
        currentPose = getTrackingPoseToBridgePark(currentPose, alliance);
        Log.d(logTag, "~~~~~~~~~~~~~parking under bridge");
        endPose = travelToNextPose(currentPose, motorList);     //Actually drive
        Log.d(logTag, "parked under bridge " + overallTime.toString());

        //  Verify all drive motors are stopped
        if(!simulateMotors) stopMotors(motorList);

        Log.d(logTag, "Closing camera");
        //  Close vuforia webcam interface
        if (tfod != null) {
            tfod.shutdown();
        }
        writeFinalTelemetry(overallTime);
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
