package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.eBotsMotionController.moveToTargetPose;

@Autonomous
@Disabled
public class E3_ASYNC_TEST extends eBotsAuton2019 {



    @Override
    public void runOpMode(){
        boolean debugOn = true;
        String logTag = "BTI_E3_UsingConfig";

        //***************************************************************
        // ******    CONFIGURATION PARAMETERS
        // ***************************************************************/
        simulateMotors = true;


        speedConfig = Speed.SLOW;
        gyroConfig = GyroSetting.NONE;
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


        //  Create the first TrackingPose
        //  Note, setting the initialGyroOffset is critical
        TrackingPose trackingPose = getFirstTrackingPose();


        //***************************************************************
        // Initiate the concurrent process to scan the quarry
        //***************************************************************
        ObserveQuarryDuringWait observeQuarryDuringWait;
        Thread thread;

        observeQuarryDuringWait = new ObserveQuarryDuringWait();
        thread = new Thread(observeQuarryDuringWait);
        thread.start();

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        endInitTelemetry();
        waitForStart();

        observeQuarryDuringWait.keepRunning = false;
        thread.interrupt();

        overallTime = new StopWatch();

        if(QuarryStone.getFoundSkyStoneCount() == 0){
            if(QuarryStone.getCountObservedStones() <= 1){
                if(debugOn) Log.d(logTag, "Must Move to second position and surveil");
            } else {
                ArrayList<QuarryStone> observedStones = QuarryStone.getObservedStones();
                if (observedStones.size() > 1) {
                    if(debugOn) Log.d(logTag, "Infer the third stone");
                }
            }
        }else {
            if(debugOn) Log.d(logTag, "Stones are already found!!");
        }

        //  Verify all drive motors are stopped
        if(!simulateMotors) stopMotors();

        telemetry.clear();
        while (overallTime.getElapsedTimeMillis() < 5000){
            telemetry.addLine("Execution Continued");
            telemetry.addData("elapsed millis", overallTime.getElapsedTimeMillis());
            telemetry.addData("elapsed millis", overallTime.getElapsedTimeMillis());
            telemetry.addData("SkyStones written to Array", QuarryStone.getFoundSkyStoneCount());

            telemetry.update();
        }
        if (debugOn) Log.d(logTag, "OpMode Complete, Overall Time: " + overallTime.toString());
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
        long startTime =timer.getElapsedTimeMillis();
        int spoofDataSet = 0;
        String logTag = "BTI_recQuarObs";
        QuarryStone currentStone;
        QuarryStone anotherStone = null;
        if (startTime < 5000) {
            currentStone = null;
            anotherStone = secondStone;
        } else if (startTime < 10000) {
            spoofDataSet = 1;
            currentStone = firstStone;
            anotherStone = secondStone;
        } else {
            spoofDataSet = 2;
            currentStone = secondStone;
            anotherStone = firstStone;
        }
        Log.d(logTag, "Entering recordQuarryObserv at time: " + startTime + " spoofData: " + spoofDataSet);

        //  See if it is recorded as skyStone
        //  Note, this recording of QuarryStone observations updates some of the
        //  properties for the QuarryStone objects.
        //  Notably:
        //      *  It purges expired observations from the time stamped observation arrays
        //      *  It populates the numObservations field based on the count of time-stamped entries
        //      *  It populates the numIdSkyStone field based on how many of those remaining observations saw it as a SkyStone
        //      These allow for simple calculation view isSkyStone() later
        boolean curStoneFlag = true;
        //  flip the flag for condition 2 (after 10 seconds)
        if (spoofDataSet == 2) curStoneFlag = false;
        //Only report 1 stone in first 5 seconds
        if (spoofDataSet!=0) currentStone.recordTSObservation(timer.getElapsedTimeMillis(), curStoneFlag);
        anotherStone.recordTSObservation(timer.getElapsedTimeMillis(), false);
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
            if (debugOn) Log.d(logTag, "Count of assignObservedQuarryStones: " + observedQuarryStones.size());
            if (debugOn) {
                for(QuarryStone stone: observedQuarryStones){
                    Log.d(logTag, stone.toString());
                }
            }
            int i = 1;
            while (keepRunning) {

                if(debugOn) Log.d(logTag, "Entering loop, pass " + i + " time: " + observationTimer.getElapsedTimeMillis());
                //Scan the quarry
                //surveilQuarry(alliance, trackingPose, new StopWatch());
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
                telemetry.update();

                if(debugOn) Log.d(logTag, "loop: " + i + " totalObs: " + QuarryStone.getTotalObservations()
                            + " Stones Obs: " + QuarryStone.getCountObservedStones()
                            + " SkyStones Obs: " + QuarryStone.getCurrentCountSkyStones()
                            + " Written to Array: " + QuarryStone.getFoundSkyStoneCount());

                //If the skyStone has been observed, set the pattern
                //TODO: Must clear skystones written if currentCountSkyStones is zero

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
                }


                try{
                    Thread.sleep(1500);
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
