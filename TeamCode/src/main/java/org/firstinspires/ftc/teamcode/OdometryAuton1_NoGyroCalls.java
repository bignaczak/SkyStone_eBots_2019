package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

import static java.lang.String.format;

@Autonomous
@Disabled
public class OdometryAuton1_NoGyroCalls extends eBotsOpMode2019 {

    private Boolean useGyroForNavigation = false;
    private Integer gyroCallFrequency = 1;
    private Double saturationLimit = 0.4;

    private Boolean simulateMotors = false;
    private Double pSignal = 0.0;
    private Double iSignal = 0.0;
    private Double computedSignal=0.0;
    private Double outputSignal=0.0;



    @Override
    public void runOpMode(){
        EncoderTracker.purgeExistingEncoderTrackers();
        EncoderTracker forwardTracker;
        EncoderTracker lateralTracker;
        //ArrayList<DcMotor> motorList = new ArrayList<>();
        ArrayList<DcMotor> motorList = new ArrayList<>();
        VirtualEncoder virForwardEncoder = new VirtualEncoder();  //if using virtual
        VirtualEncoder virLateralEncoder = new VirtualEncoder();  //if using virtual
        Log.d("BTI_runOpMode", "Initializing...");
        if (simulateMotors) {
            forwardTracker = new EncoderTracker(virForwardEncoder, EncoderTracker.RobotOrientation.FORWARD);
            lateralTracker = new EncoderTracker(virLateralEncoder, EncoderTracker.RobotOrientation.LATERAL);


        } else {
            initializeDriveMotors(motorList, true);
            forwardTracker = new EncoderTracker(motorList.get(0), EncoderTracker.RobotOrientation.FORWARD);
            lateralTracker = new EncoderTracker(motorList.get(1), EncoderTracker.RobotOrientation.LATERAL);

        }
        //  Set starting pose and capture gyro offset
        final Pose startingPose = new Pose(Pose.StartingPose.RED_FOUNDATION);

        //  Set target pose
        //final Pose targetPose = new Pose(startingPose.getX(), 0.0, startingPose.getHeading());
        final Pose targetPose = new Pose(0.0, 0.0, startingPose.getHeading());
        //final Pose targetPose = new Pose(-startingPose.getX(), -startingPose.getY(), startingPose.getHeading());

        final double pGain = 0.2;
        final double iGain = 0.0; //0.005;

        if (useGyroForNavigation) initializeImu();
        Integer loopCount = 0;

        Double previousSignalSign=0.0;

        Boolean isSaturated = true;
        Boolean driveSignalSignChange = false;  //  When heading is locked, the headingError only
                                        //  gets recalculated when driveSignal changes sign
                                        //  This allows for the I portion of the PID controller
                                        //  to overshoot the target position


        //This is the angle the robot needs to travel relative to the robot's reference frame
        //This assumes that forward is 0 degress, 90 degrees is traveling left
        double robotTravelAngleRad;


        //These values get calculated by calculateDriveVector function
        double[] driveValues = new double[4];  //Array of values for the motor power levels
        double maxValue;


        //  *********  INITIALIZE FOR FIRST PASS THROUGH LOOP   *****************
        //  Create currentPose, which is the tracked position from start to target
        //  This is a special type of pose that is intended to track the path of travel
        //  It has a targetPose, which is the intended destination
        //  It also has an error object which tracks it's status relative to targetPose
        TrackingPose currentPose = new TrackingPose(startingPose, targetPose);

        //This is called only once to document offset of gyro from field coordinate system
        if(useGyroForNavigation) {
            currentPose.setInitialGyroOffset(getGyroReadingDegrees());
        } else {
            currentPose.setInitialGyroOffset(0.0);
        }

        StopWatch stopWatch = new StopWatch();
        String loopMetrics = stopWatch.toString(loopCount);
        //Double loopCheckSum = currentPose.getErrorMagnitude() + Math.abs(currentPose.getErrorSum());
        writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);

        //***************************************************************
        //  END OF OPMODE INITIALIZATION
        //  Wait for the game to start(driver presses PLAY)
        //***************************************************************
        waitForStart();
        //Setup the time increment for autonomous


        stopWatch.startTimer();
        Long loopStartTime; //This will get initialized at start of loop
        Long loopEndTime = stopWatch.getElapsedTimeMillis();  //Grabs the start of timer
        Long loopDuration=0L;  //Initialized right before assigning drive motors

        //while(opModeIsActive() && currentPose.getErrorControlValue() > 0.5 && loopCount < 5000){
        while(opModeIsActive() && currentPose.getErrorMagnitude() > 0.2){
        //while(opModeIsActive() && loopCount<299){
            Log.d("BTI_ruOpMode", "*******" + loopCount.toString());
            Log.d("BTI_ruOpMode", "numEncoders" + EncoderTracker.getEncoderTrackerCount());


            loopStartTime = loopEndTime;        //Uses the end time of the previous loop for start time (ensures continuity of cycles)

            if (loopCount>0){  //For every iteration after the first
                if(simulateMotors) {
                    //virForwardEncoder.simulateLoopOutput(outputSignal,loopDuration, currentPose);
                    //virLateralEncoder.simulateLoopOutput(outputSignal,loopDuration, currentPose);
                    //EncoderTracker.updateVirtualEncoders(outputSignal,loopDuration, currentPose);
                }
                EncoderTracker.getNewPose(currentPose);               //update position if not first loop

                if (useGyroForNavigation && (loopCount % gyroCallFrequency == 0)) {
                    currentPose.setHeadingFromGyro(getGyroReadingDegrees());  //Update heading with robot orientation
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
                    signChangeDebug = "No Sign Change";
                } else {
                    driveSignalSignChange = true;
                    signChangeDebug = "Yes Sign Change";
                }


            currentPose.setHeadingErrorLocked(isSaturated, driveSignalSignChange);
            currentPose.updateTravelDirection();
            outputSignal = (isSaturated) ? saturationLimit : Math.abs(computedSignal);  //Note: always positive unlike computed signal


            Log.d("BTI_ruOpMode", printDriveSignalMetrics() + " " + signChangeDebug);


            //Which way the robot needs to travel relative to the robot forward direction
            robotTravelAngleRad = currentPose.getGyroDriveDirectionRad();

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
                //Now actually assign the calculated drive values to the motors in motorList
                //Log.d("BTI_BeforeSet", printActualMotorPowers(motorList));

                maxValue = findMaxAbsValue(driveValues);  //See what the max drive value is set to
                scaleDrive(saturationLimit/maxValue, driveValues);  //If max value isn't 1, Scale the values up so max is 1

                Log.d("BTI_Instructions", printMotorDriveInstructions(motorList, driveValues));

                int i=0;
                for (DcMotor m: motorList){
                    m.setPower(driveValues[i]);
                    i++;
                }


                //Log.d("BTI_AfterSet", printActualMotorPowers(motorList));

            }

            loopCount++;
            //loopCheckSum = currentPose.getErrorMagnitude() + Math.abs(currentPose.getErrorSum());
            loopMetrics = stopWatch.toString(loopCount);
            writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);
        }

        if(!simulateMotors) stopMotors(motorList);

        //Once the cycle is completed, hold the telemetry for 5 seconds
        StopWatch timer = new StopWatch();
        timer.startTimer();
        loopMetrics = stopWatch.toString(loopCount);

        while (opModeIsActive() && timer.getElapsedTimeSeconds() < 5){
            writeOdometryTelemetry(loopMetrics, currentPose, forwardTracker, lateralTracker);
        }
        //stop();
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



}
