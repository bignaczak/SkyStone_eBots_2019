package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

import static java.lang.String.format;

@Autonomous
public class OdometryAuton_MultiStep extends eBotsOpMode2019 {

    //***************************************************************88
    //******    CONFIGURATION PARAMETERS
    //***************************************************************88

    private Boolean simulateMotors = false;
    private Boolean useGyroForNavigation = true;
    private Integer gyroCallFrequency = 1;
    private Double saturationLimit = 0.5;
    private final double pGain = 0.2;
    private final double iGain = 0.0;  //0.005;

    //***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88

    private Double pSignal = 0.0;
    private Double iSignal = 0.0;
    private Double computedSignal=0.0;
    private Double outputSignal=0.0;

    private EncoderTracker forwardTracker;
    private EncoderTracker lateralTracker;
    private ArrayList<Pose> wayPoses;



    @Override
    public void runOpMode(){
        EncoderTracker.purgeExistingEncoderTrackers();      //Clean out any pre-existing encoders

        ArrayList<DcMotor> motorList = new ArrayList<>();   //List of motors

        //  Setup encoders, either simulated or real
        if (simulateMotors) {
            initializeEncoderTrackers();        //virtual encoders
        }else{
            initializeDriveMotors(motorList, true);
            initializeEncoderTrackers(motorList);           //actual encoders
        }



        Integer wayPoseIndex = 1;
        wayPoses = new ArrayList<>();
        wayPoses.clear();       // get rid of pre-existing poses

        setWayPoses(wayPoses);
        //  *********  INITIALIZE FOR FIRST PASS THROUGH LOOP   *****************
        //  Create currentPose, which is the tracked position from start to target
        //  This is a special type of pose that is intended to track the path of travel
        //  It has a targetPose, which is the intended destination
        //  It also has an error object which tracks it's status relative to targetPose
        TrackingPose currentPose = new TrackingPose(wayPoses.get(0), wayPoses.get(wayPoseIndex));

        //  Instantiate gyro object if being utilized
        //This is called only once to document offset of gyro from field coordinate system

        if (useGyroForNavigation) {
            initializeImu();
            currentPose.setInitialGyroOffset(getCurrentHeading());  //This is called only once to document offset of gyro from field coordinate system
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }


        String loopMetrics = "Loop Initialized";
        //Double loopCheckSum = currentPose.getErrorMagnitude() + Math.abs(currentPose.getErrorSum());
        writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();

        //Travel first leg
        Log.d("BTI_runOpMode", "~~~~~~~~~~~~~Starting LegStarting Leg " + wayPoseIndex.toString());
        TrackingPose endPose = travelToNextPose(currentPose, motorList);
        Double previousInitialGyroOffset = currentPose.getInitialGyroOffset();
        if(!simulateMotors) stopMotors(motorList);
        wayPoseIndex++;

        while(wayPoseIndex< wayPoses.size()) {
            //Set ending of previous leg as the starting pose for new leg
            Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
            currentPose = new TrackingPose(startPose, wayPoses.get(wayPoseIndex));
            setInitialOffsetForTrackingPose(imu, currentPose, previousInitialGyroOffset);

            Log.d("BTI_runOpMode", "~~~~~~~~~~~~~Starting Leg " + wayPoseIndex.toString());
            endPose = travelToNextPose(currentPose, motorList);
            if(!simulateMotors) stopMotors(motorList);
            wayPoseIndex++;
        }

        //Once the cycle is completed, hold the telemetry for 5 seconds
        StopWatch timer = new StopWatch();
        timer.startTimer();
        loopMetrics = "All Points Completed";

        while (opModeIsActive() && timer.getElapsedTimeSeconds() < 5){
            writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);
        }
    }

    private void writeOdometryTelemetry(String loopMetrics, TrackingPose currentPose, EncoderTracker encoder1, EncoderTracker encoder2) {
        telemetry.addData("Loop Metrics: ", loopMetrics);

        telemetry.addData("currentPose: ", currentPose.toString());
        telemetry.addData("targetPose: ", currentPose.getTargetPose().toString());
        telemetry.addData("currentError: ", currentPose.printError());

        telemetry.addData("Loop Error Condition Check: ", format("%.3f", currentPose.getErrorControlValue()));
        telemetry.addLine(printDriveSignalMetrics());

        telemetry.addData("Heading Locked: ", currentPose.isHeadingErrorLocked());
        telemetry.addData("forwardTracker: ", encoder1.toString());
        telemetry.addData("lateralTracker: ", encoder2.toString());
        telemetry.update();
    }

    public String printDriveSignalMetrics(){
        return "pSig/iSig/outSig: " + format("%.3f", pSignal) + " / " + format("%.3f", iSignal) + " / " + format("%.3f" ,outputSignal);
    }

    private void initializeEncoderTrackers(){
        //Initialize virtual encoders
        EncoderTracker.purgeExistingEncoderTrackers();
        Log.d("initEncoderTrackersVir", "Initializing Virtual Encoders");
        VirtualEncoder virForwardEncoder = new VirtualEncoder();  //if using virtual
        VirtualEncoder virLateralEncoder = new VirtualEncoder();  //if using virtual
        forwardTracker = new EncoderTracker(virForwardEncoder, EncoderTracker.RobotOrientation.FORWARD);
        lateralTracker = new EncoderTracker(virLateralEncoder, EncoderTracker.RobotOrientation.LATERAL);
    }

    private void initializeEncoderTrackers(ArrayList<DcMotor> motorList){
        //Initialize actual encoders
        Log.d("initEncoderTrackersReal", "Initializing Real Encoders");
        forwardTracker = new EncoderTracker(motorList.get(0), EncoderTracker.RobotOrientation.FORWARD);
        lateralTracker = new EncoderTracker(motorList.get(1), EncoderTracker.RobotOrientation.LATERAL);
    }

    private void setWayPoses(){
        wayPoses = new ArrayList<>();
        wayPoses.clear();       // get rid of pre-existing poses

        wayPoses.add(new Pose(Pose.StartingPose.RED_FOUNDATION));
        //travel in the positive Y direction to midline
        wayPoses.add(new Pose(Pose.StartingPose.RED_FOUNDATION.getxStart(), 0.0, Pose.StartingPose.RED_FOUNDATION.getHeadingStart()));
        //travel back 12 inches
        wayPoses.add(new Pose(Pose.StartingPose.RED_FOUNDATION.getxStart(), -12.0, Pose.StartingPose.RED_FOUNDATION.getHeadingStart()));
        //travel to blue foundation start point
        wayPoses.add(new Pose(Pose.StartingPose.BLUE_FOUNDATION.getxStart(), Pose.StartingPose.BLUE_FOUNDATION.getyStart(), 90.0));
    }

    private TrackingPose travelToNextPose(TrackingPose currentPose, ArrayList<DcMotor> motorList){
        Integer loopCount = 0;
        String loopMetrics;
        Double previousSignalSign=0.0;
        Boolean isSaturated = true;
        Boolean driveSignalSignChange = false;  //  When heading is locked, the headingError only
                                                //  gets recalculated when driveSignal changes sign
                                                //  This allows for the I portion of the PID controller
                                                //  to overshoot the target position

        StopWatch stopWatch = new StopWatch();
        stopWatch.startTimer();
        Long loopStartTime; //This will get initialized at start of loop
        Long loopEndTime = stopWatch.getElapsedTimeMillis();  //Grabs the start of timer
        Long loopDuration=0L;  //Initialized right before assigning drive motors

        //while(opModeIsActive() && currentPose.getErrorControlValue() > 0.5 && loopCount < 5000){
        while(opModeIsActive() && currentPose.getErrorMagnitude() > 0.2){
            //while(opModeIsActive() && loopCount<299){
            Log.d("travelToNextPose", "*******" + loopCount.toString());
            Log.d("travelToNextPose", "numEncoders" + EncoderTracker.getEncoderTrackerCount());

            double[] driveValues = new double[4];               //Array of values for the motor power levels
            double maxValue;

            loopStartTime = loopEndTime;        //Uses the end time of the previous loop for start time (ensures continuity of cycles)

            if (loopCount>0){  //For every iteration after the first
                EncoderTracker.getNewPose(currentPose);               //update position if not first loop

                if (useGyroForNavigation && (loopCount % gyroCallFrequency == 0)) {
                    currentPose.setHeadingFromGyro(getCurrentHeading());  //Update heading with robot orientation
                }
                currentPose.calculatePoseError();                     //Update error object

                //Compute a new ErrorSum for the Integrator
                //  if BOTH of the following conditions are met, don't add the integrator
                //  -->signal can't be saturated in previous iteration (!isSaturated)
                //  -->error sign same as error Sum (evaluates to true if errorSum 0)
                currentPose.updateErrorSum(isSaturated);
            }

            //pSignal = pGain * currentPose.getErrorMagnitude();
            pSignal = pGain * currentPose.getSignedError();  //Sign is important as the integrator unwinds
            //  During overshoot, need proportional signal to negate errorSum

            iSignal = iGain * currentPose.getErrorSum();  //Note: this can be negative

            previousSignalSign = Math.signum(computedSignal);
            computedSignal = pSignal + iSignal;     //This can be negative and sign is important

            isSaturated = Math.abs(computedSignal) > saturationLimit ? true : false;  //consider magnitude only

            String signChangeDebug;
            if (    loopCount == 0 |
                    (previousSignalSign == Math.signum(computedSignal))) {

                driveSignalSignChange = false;
                signChangeDebug = "[No Sign Change]";
            } else {
                driveSignalSignChange = true;
                signChangeDebug = "[**Yes** Sign Change]";
            }


            currentPose.setHeadingErrorLocked(isSaturated, driveSignalSignChange);
            currentPose.updateTravelDirection();
            outputSignal = (isSaturated) ? saturationLimit : Math.abs(computedSignal);  //Note: always positive unlike computed signal


            Log.d("BTI_ruOpMode", printDriveSignalMetrics() + "--" + signChangeDebug);


            calculateFieldOrientedDriveVector(currentPose.getTravelDirectionRad(), currentPose.getHeadingRad(),outputSignal,0,driveValues);

            //Add a delay if using simulated devices
            //if (simulateMotors) sleep(100);

            loopEndTime = stopWatch.getElapsedTimeMillis();
            loopDuration = loopEndTime - loopStartTime;
            //Apply drive vector to motors or simulated encoders

            if(simulateMotors){
                //virForwardEncoder.simulateLoopOutput(outputSignal,loopDuration, currentPose);
                EncoderTracker.updateVirtualEncoders(outputSignal,loopDuration, currentPose);

            } else {
                maxValue = findMaxAbsValue(driveValues);  //See what the max drive value is set to
                scaleDrive(saturationLimit/maxValue, driveValues);  //If max value isn't 1, Scale the values up so max is 1

                //Now actually assign the calculated drive values to the motors in motorList
                for (int i = 0; i < motorList.size(); i++) {
                    motorList.get(i).setPower(driveValues[i]);
                }
            }

            loopCount++;
            //loopCheckSum = currentPose.getErrorMagnitude() + Math.abs(currentPose.getErrorSum());
            loopMetrics = stopWatch.toString(loopCount);
            writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);
        }

        return currentPose;
    }

    public void setInitialOffsetForTrackingPose(BNO055IMU imu, TrackingPose trackingPose, Double previousInitialGyroOffset){
        //If using heading, update the heading using the gyro reading
        Double gyroReading;
        if (useGyroForNavigation) {
            gyroReading = getCurrentHeading();
            trackingPose.setHeadingFromGyro(gyroReading, previousInitialGyroOffset);
        } else{
            gyroReading = trackingPose.getHeading() + previousInitialGyroOffset;
        }
        trackingPose.setInitialGyroOffset(gyroReading);

    }
}
