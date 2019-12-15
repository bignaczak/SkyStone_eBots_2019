package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.ListIterator;

@Autonomous
@Disabled
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
        setAllianceObjects();           //Create objects for Quarry Stones and Foundation
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
