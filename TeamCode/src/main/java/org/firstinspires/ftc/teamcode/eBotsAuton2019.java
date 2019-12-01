package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.concurrent.ThreadLocalRandom;

import static java.lang.String.format;

public abstract class eBotsAuton2019 extends eBotsOpMode2019 {

    /*****************************************************************
    //******    CONFIGURATION PARAMETERS
    //****************************************************************/

    protected Boolean simulateMotors = false;
    protected Boolean useGyroForNavigation;
    protected int gyroCallFrequency;
    protected double saturationLimit;
    protected double turnSpeed;
    protected double pGain;
    protected double iGain;


    protected Boolean useSoftStart;
    protected Long softStartDurationMillis;
    protected Double minPowerLimit;

    protected Double headingAngleTolerance;
    protected Double positionalTolerance;
    protected Double integratorUnwindLimit;

    protected static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    protected static final String LABEL_FIRST_ELEMENT = "Stone";
    protected static final String LABEL_SECOND_ELEMENT = "Skystone";
    protected static final String WEBCAM_NAME = "Webcam 1";

    protected static final double offsetDistance = 18.0;      //How far away from the stone should the robot be?

    /*****************************************************************
    //******    CLASS VARIABLES
    //****************************************************************/

    protected Double pSignal = 0.0;
    protected Double iSignal = 0.0;
    protected Double computedSignal=0.0;
    protected Double outputSignal=0.0;

    protected EncoderTracker forwardTracker;
    protected EncoderTracker forwardTracker2;
    protected EncoderTracker lateralTracker;
    protected ArrayList<Pose> wayPoses;
    protected Foundation foundation;
    /****************************************************************/

    /*****************************************************************
     //******    GETTERS AND SETTERS
     //****************************************************************/
    public double getIGain(){return iGain;}
    public double getPGain(){return pGain;}

    /*****************************************************************
    //******    ENUMERATIONS
    //****************************************************************/

    public enum Alliance{
        RED
        , BLUE
        , NONE
    }

    public enum FieldSide{
        FOUNDATION
        , QUARRY
        , TEST
        , FOUNDATION_V2
    }

    public enum DelayedStart{
        NO,
        YES
    }

    public enum Speed{
        SLOW (0.35, 0.3, 0.15, 0.0, 0.0, 0.02, 0.00, 0.0),
        MEDIUM (0.60,0.4,  0.07, 0.015, 0.0, 0.04, 0.01, 0.0),
        FAST (0.8, 0.4, 0.04, 0.008, 0.0,0.05, 0.005,0.0);

        /**  ENUM VARIABLES     **************/
        private double maxSpeed;
        private double turnSpeed;
        private double k_p;  //for translate proportional
        private double k_i;  //for translate integrator
        private double k_d;  //for translate derivative
        private double s_p;  //for spin proportional
        private double s_i;  //for spin integrator
        private double s_d;  //for spin derivative
        /**  CONSTRUCTOR    **************/
        Speed(double speed, double turnSpeed, double pGain, double iGain, double dGain, double spinPGain, double spinIGain, double spinDGain){
            this.maxSpeed = speed;
            this.turnSpeed = turnSpeed;
            this.k_p = pGain;
            this.k_i = iGain;
            this.k_d = dGain;
            this.s_p = spinPGain;
            this.s_i = spinIGain;
            this.s_d = spinDGain;
        }
        /**  ENUM GETTERS AND SETTERS  ***********/
        public double getMaxSpeed(){return this.maxSpeed;}
        public double getTurnSpeed(){return this.turnSpeed;}
        public double getK_p(){return this.k_p;}
        public double getK_i(){return this.k_i;}
        public double getS_p(){return this.s_p;}
        public double getS_i(){return this.s_i;}

        @Override
        public String toString(){
            return "maxSpeed: " + maxSpeed + " , turnSpeed: " + turnSpeed
                    + ", Translate Coefficients k_p / k_i / k_d: " + this.k_p + " / " + this.k_i + " / " + this.k_d
                    + ", Spin Coefficients s_p / s_i / s_d: " + this.s_p + " / " + this.s_i + " / " + this.s_d;
        }
    }

    public enum GyroSetting{
        NONE(false, -1),
        INFREQUENT (true, 25),
        EVERY_LOOP (true, 1);

        /**  ENUM VARIABLES     **************/
        private Boolean gyroOn;
        private Integer readFrequency;

        /**  CONSTRUCTOR    **************/
        GyroSetting (Boolean gyroPower, Integer frequency){
            this.gyroOn = gyroPower;
            this.readFrequency = frequency;
        }

        /**  ENUM GETTERS AND SETTERS  ***********/
        public Boolean isGyroOn(){return this.gyroOn;}
        public Integer getReadFrequency(){return this.readFrequency;}
    }

    public enum SoftStart{
        NO (false, 0L, 0.2),
        MEDIUM (true, 750L, 0.2),
        SLOW_START(true, 1200L, 0.2);

        /**  ENUM VARIABLES     **************/
        private Boolean softStartOn;
        private Long durationMillis;
        private Double minPower;

        /**  CONSTRUCTOR    **************/
        SoftStart(Boolean softStartOn, Long durationMillis, Double minPower){
            this.softStartOn = softStartOn;
            this.durationMillis = durationMillis;
            this.minPower = minPower;

        }
        /**  ENUM GETTERS AND SETTERS  ***********/
        public Boolean isSoftStartOn(){return this.softStartOn;}
        public Long getDurationMillis(){return  this.durationMillis;}
        public Double getMinPower(){return minPower;}
    }

    public enum Accuracy{
        LOOSE (10.0, 0.5, 500.0, 500.0),
        STANDARD (5.0, 0.5, 10.0, 1.0),
        TIGHT (2.5, 0.1, 25.0, 25.0);

        /**  ENUM VARIABLES     **************/

        private Double headingAngleAccuracy;
        private Double positionalAccuracy;
        private Double integratorUnwindLimit;
        private Double spinIntegratorUnwindLimit;

        /**  CONSTRUCTOR    **************/

        Accuracy(Double headingAngleAccuracy, Double positionalAccuracy, Double integratorUnwind, Double spinIntegratorUnwind){
            this.headingAngleAccuracy = headingAngleAccuracy;
            this.positionalAccuracy = positionalAccuracy;
            this.integratorUnwindLimit = integratorUnwind;  // for translate position
            this.spinIntegratorUnwindLimit = spinIntegratorUnwind;  // for translate position
        }

        /**  ENUM GETTERS AND SETTERS  ***********/

        public Double getHeadingAngleAccuracy() {
            return headingAngleAccuracy;
        }
        public Double getPositionalAccuracy() {
            return positionalAccuracy;
        }
        public Double getIntegratorUnwindLimit() {
            return integratorUnwindLimit;
        }
        public Double getSpinIntegratorUnwindLimit() {
            return spinIntegratorUnwindLimit;
        }
        @Override
        public String toString(){
            return "Translate position tol: " + format("%.1f",positionalAccuracy)
                    + ", Translate Integrator: " + format("%.1f", integratorUnwindLimit)
                    + ", Heading angle tol: " + format("%.1f", headingAngleAccuracy)
                    + ", Heading Integrator: " + format("%.1f", spinIntegratorUnwindLimit);
        }
    }

    public enum FieldObject{
        QUARRY_STONE (6.25, 7.25),
        FOUNDATION (4.0, 5.0);

        private double minDistance;
        private double maxDistance;

        public double getMinDistance(){return this.minDistance;}
        public double getMaxDistance(){return this.maxDistance;}

        FieldObject(double minDistanceIn, double maxDistanceIn){
            this.minDistance = minDistanceIn;
            this.maxDistance = maxDistanceIn;
        }
    }

    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/


    /*****************************************************************
     //******    CLASS METHODS
     //****************************************************************/

    protected void setWayPoses(ArrayList<Pose> wayPoses, Alliance alliance, FieldSide fieldSide, DelayedStart delayedStart) {
        /**  This function sets the wayPoses for blue side and then applies a
         * transformation by mirroring about the X axis for position and pose
         */

        if (wayPoses.size() > 0) wayPoses.clear();       // get rid of pre-existing poses

        //These waypoints are for the blue side
        if (delayedStart == DelayedStart.YES) {
            //  Execute a simple opMode where the robot moves minimally after extending arm
            //  Set the starting Point assuming on BLUE FOUNDATION
            wayPoses.add(new Pose(24.0, 60.0, 0.0));
            //  Now if Quarry side, flip the sign of the s coordination and flip heading
            //  Note that if red side, the coordinates will be transformed again
            if (fieldSide == FieldSide.QUARRY) {
                Pose p = wayPoses.get(0);
                p.setX(-p.getX());      //Flip sign for Y
                p.setHeading(p.getHeading()+180.0);     //Rotate from 0 to 180 deg
            }
            wayPoses.add(new Pose(0.0, 60.0, 0.0));


        } else if (fieldSide == FieldSide.FOUNDATION) {
            //Blue Foundation
            wayPoses.add(new Pose(Pose.StartingPose.FOUNDATION));
            //Move to foundation plate and lower rake
            wayPoses.add(new Pose(50.0, 34.0, 90.0, Pose.PostMoveActivity.LOWER_RAKE));
            //drag foundation over to wall
            wayPoses.add(new Pose(50.0, 60.0, 90.0, Pose.PostMoveActivity.RAISE_RAKE));
            //move sideways towards center of field
            wayPoses.add(new Pose(18.0, 60.0, 90.0));
            //move backwards towards center and raise arm, then lower for driving under bridge
            wayPoses.add(new Pose(18.0, 55.0, 0.0, Pose.PostMoveActivity.EXTEND_ARM_THEN_LOWER_LIFTER));
            //Push foundation against back wall
            wayPoses.add(new Pose(28.5, 55.0, 0.0));
            //move back to wall
            wayPoses.add(new Pose(25.0, 60.0, 0.0));
            //Stay in same location but spin to be skinny under bridge
            wayPoses.add(new Pose(0.0, 60.0, 0.0));

        } else if (fieldSide == FieldSide.QUARRY) {
            // Start right next to depot
            wayPoses.add(new Pose(Pose.StartingPose.QUARRY));
            //Move to position to scan stones 4 and 5 and extend arm
            wayPoses.add(new Pose(-30.0, 60.0, -90.0, Pose.PostMoveActivity.EXTEND_ARM_THEN_LOWER_LIFTER));
            // Stay in the same place and scan quarry
            wayPoses.add(new Pose(-30.0, 60.0, -90.0, Pose.PostMoveActivity.SCAN_FOR_SKYSTONE));

        } else if (fieldSide == FieldSide.FOUNDATION_V2) {
            //   FOUNDATION_V2 IS NOW THE SAME AS FOUNDATION
            //Blue Foundation
            wayPoses.add(new Pose(Pose.StartingPose.FOUNDATION));
            //Move to foundation plate and lower rake
            wayPoses.add(new Pose(50.0, 34.0, 90.0, Pose.PostMoveActivity.LOWER_RAKE));
            //drag foundation over to wall
            wayPoses.add(new Pose(50.0, 60.0, 90.0, Pose.PostMoveActivity.RAISE_RAKE));
            //move sideways towards center of field
            wayPoses.add(new Pose(18.0, 60.0, 90.0));
            //move backwards towards center and raise arm, then lower for driving under bridge
            wayPoses.add(new Pose(18.0, 55.0, 0.0, Pose.PostMoveActivity.EXTEND_ARM_THEN_LOWER_LIFTER));
            //Push foundation against back wall
            wayPoses.add(new Pose(28.5, 55.0, 0.0));
            //move back to wall
            wayPoses.add(new Pose(25.0, 60.0, 0.0));
            //Stay in same location but spin to be skinny under bridge
            wayPoses.add(new Pose(0.0, 60.0, 0.0));

            //Park in the middle
            //wayPoses.add(new Pose(-48.0, 0.0, 180.0));
        } else {
            //start at the origin facing X axis
            wayPoses.add(new Pose(0.0, 0.0, 90.0));
            //move forward 20 inches
            wayPoses.add(new Pose(0.0, -5.0, 90.0));
            //move back to origin
            wayPoses.add(new Pose(0.0, -10.0, 90.0));
            //move back to origin
            wayPoses.add(new Pose(0.0, -15.0, 90.0));
        }

        //  Apply the transformation.
        //  -->  Change sign of Y position
        //  -->  Change sign of Heading
        if (alliance == Alliance.RED) {
            for (Pose p : wayPoses) {
                applyRedAlliancePoseTransform(p);
            }
        }
    }
    protected void setGyroConfiguration(GyroSetting gyroSetting){
        useGyroForNavigation = gyroSetting.isGyroOn();
        gyroCallFrequency = gyroSetting.getReadFrequency();

        if (useGyroForNavigation) initializeImu();
    }

    protected void setSpeedConfiguration(Speed speedConfig){
        saturationLimit = speedConfig.getMaxSpeed();
        turnSpeed = speedConfig.getTurnSpeed();
        pGain = speedConfig.getK_p();
        iGain = speedConfig.getK_i();
    }

    protected void setSoftStartConfig(SoftStart softStartConfig){
        useSoftStart = softStartConfig.isSoftStartOn();
        softStartDurationMillis = softStartConfig.getDurationMillis();
        minPowerLimit = softStartConfig.getMinPower();
    }

    protected void setAccuracyLimits(Accuracy accuracy){
        headingAngleTolerance = accuracy.getHeadingAngleAccuracy();
        positionalTolerance = accuracy.getPositionalAccuracy();
        integratorUnwindLimit = accuracy.getIntegratorUnwindLimit();
    }

    protected void setAllianceObjects(Alliance alliance){
        foundation = new Foundation(alliance);
        //***************************************************************
        //  Build the QuarryStone objects representing the 6 positions
        //***************************************************************
        QuarryStone.constructQuarry(alliance);

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

    public String printDriveSignalMetrics(){
        return "pSig/iSig/outSig: " + format("%.3f", pSignal) + " / " + format("%.3f", iSignal) + " / " + format("%.3f" ,outputSignal);
    }

    protected void initializeEncoderTrackers(){
        //Initialize virtual encoders
        EncoderTracker.purgeExistingEncoderTrackers();
        Log.d("initEncoderTrackersVir", "Initializing Virtual Encoders");
        VirtualEncoder virForwardEncoder = new VirtualEncoder();  //if using virtual
        VirtualEncoder virForwardEncoder2 = new VirtualEncoder();  //if using virtual
        VirtualEncoder virLateralEncoder = new VirtualEncoder();  //if using virtual
        forwardTracker = new EncoderTracker(virForwardEncoder, EncoderTracker.RobotOrientation.FORWARD);
        forwardTracker2 = new EncoderTracker(virForwardEncoder2, EncoderTracker.RobotOrientation.FORWARD);
        forwardTracker2.setSpinBehavior(EncoderTracker.SpinBehavior.DECREASES_WITH_ANGLE);
        forwardTracker2.setClickDirection(EncoderTracker.ClickDirection.REVERSE);
        lateralTracker = new EncoderTracker(virLateralEncoder, EncoderTracker.RobotOrientation.LATERAL);
    }

    protected void initializeEncoderTrackers(ArrayList<DcMotor> motorList){
        //Initialize actual encoders

        EncoderTracker.purgeExistingEncoderTrackers();      //Clean out any pre-existing encoders

        Log.d("initEncoderTrackersReal", "Initializing Real Encoders");
        forwardTracker = new EncoderTracker(backRight, EncoderTracker.RobotOrientation.FORWARD);
        lateralTracker = new EncoderTracker(frontRight, EncoderTracker.RobotOrientation.LATERAL);
    }


    protected TrackingPose travelToNextPose(TrackingPose currentPose){
        boolean debugOn = false;
        Integer loopCount = 0;
        String loopMetrics;
        Double previousSignalSign=0.0;
        Boolean isSaturated = true;
        Boolean isLowPower = false;
        Boolean softStartActive = true;
        Boolean driveSignalSignChange = false;  //  When heading is locked, the headingError only
        String debugTag = "BTI_travelToNextPose";
        ArrayList<DcMotor> motorList = getDriveMotors();
        //  gets recalculated when driveSignal changes sign
        //  This allows for the I portion of the PID controller
        //  to overshoot the target position

        StopWatch stopWatch = new StopWatch();
        stopWatch.startTimer();
        Long loopStartTime; //This will get initialized at start of loop
        Long loopEndTime = stopWatch.getElapsedTimeMillis();  //Grabs the start of timer
        Long loopDuration=0L;  //Initialized right before assigning drive motors

        int forwardEncoderClicksStart = forwardTracker.getClicks();
        int lateralEncoderClicksStart = lateralTracker.getClicks();
        if (debugOn) Log.d(debugTag, "Start Position: " + currentPose.toString());
        if (debugOn) Log.d(debugTag, "Target Position: " + currentPose.getTargetPose().toString());
        if (debugOn) Log.d(debugTag, "Forward Encoder: " + forwardTracker.toString());
        if (debugOn) Log.d(debugTag, "Lateral Encoder: " + lateralTracker.toString());

        //while(opModeIsActive() && currentPose.getErrorControlValue() > 0.5 && loopCount < 5000){
        while(opModeIsActive() && currentPose.getErrorMagnitude() > positionalTolerance){
            //while(opModeIsActive() && loopCount<299){
            if (debugOn) Log.d(debugTag, "*******" + loopCount.toString());
            if (debugOn) Log.d(debugTag, "numEncoders" + EncoderTracker.getEncoderTrackerCount());

            double[] driveValues = new double[4];               //Array of values for the motor power levels
            double maxValue;

            loopStartTime = loopEndTime;        //Uses the end time of the previous loop for start time (ensures continuity of cycles)

            if (loopCount>0){  //For every iteration after the first
                if (debugOn) Log.d(debugTag, "Back in Loop");
                if (debugOn) Log.d(debugTag, currentPose.toString());
                if (debugOn) Log.d(debugTag, currentPose.printError());

                if (useGyroForNavigation && (loopCount % gyroCallFrequency == 0)) {
                    if (debugOn) Log.d(debugTag, "About to set heading from gyro...");
                    if (debugOn) Log.d(debugTag, "initialGyroOffset: " + currentPose.getInitialGyroOffset());
                    currentPose.setHeadingFromGyro(getGyroReadingDegrees());  //Update heading with robot orientation
                }
                currentPose.calculatePoseError();                     //Update error object
                if (debugOn) Log.d(debugTag, "Pose Error Calculated");

                //Compute a new ErrorSum for the Integrator
                //  if BOTH of the following conditions are met, don't add the integrator
                //  -->signal can't be saturated in previous iteration (!isSaturated)
                //  -->error sign same as error Sum (evaluates to true if errorSum 0)
                currentPose.updateErrorSum(isSaturated);
                if (debugOn) Log.d(debugTag, "Updated Error Sum");
                EncoderTracker.getNewPose(currentPose);               //update position if not first loop


            }

            pSignal = pGain * currentPose.getSignedError();  //Sign is important as the integrator unwinds

            //  During overshoot, need proportional signal to negate errorSum
            iSignal = iGain * currentPose.getErrorSum();  //Note: this can be negative

            previousSignalSign = Math.signum(computedSignal);
            computedSignal = pSignal + iSignal;     //This can be negative and sign is important

            //Set some power states for later uses
            isSaturated = Math.abs(computedSignal) > saturationLimit ? true : false;  //consider magnitude only
            isLowPower =  Math.abs(computedSignal) < minPowerLimit   ? true : false;  //power is below motor stall condition

            if (useSoftStart && stopWatch.getElapsedTimeMillis() < softStartDurationMillis){
                softStartActive = true;
            } else {
                softStartActive = false;
            }

            String signChangeDebug;
            if (    loopCount == 0 |
                    (previousSignalSign == Math.signum(computedSignal))) {

                driveSignalSignChange = false;
                signChangeDebug = "[No Sign Change]";
            } else {
                driveSignalSignChange = true;
                signChangeDebug = "[**Yes** Sign Change]";
            }

            currentPose.setHeadingErrorLocked(isSaturated, driveSignalSignChange, iGain);
            currentPose.updateTravelDirection();

            //Apply bounds to the output signal:
            //      ** Clip high end at saturationLimit
            //      ** Clip low end at minPowerLimit
            //      ** If within transient period, ramp down the power
            outputSignal = (isSaturated) ? saturationLimit : Math.abs(computedSignal);  //Note: always positive unlike computed signal
            outputSignal = (isLowPower)  ? minPowerLimit   : outputSignal;
            if (softStartActive) outputSignal = applySoftStartScaling(stopWatch);

            if (debugOn) Log.d(debugTag, printDriveSignalMetrics() + "--" + signChangeDebug);

            calculateFieldOrientedDriveVector(currentPose.getTravelDirectionRad(), currentPose.getHeadingRad(),outputSignal,0,driveValues);
            //Scale the drive vector to so the max motor speed equals the outputSignal value
            //The calculation can generate values less than outputSignal
            //This scaling provides better speed control
            maxValue = findMaxAbsValue(driveValues);  //See what the max drive value is set to
            if (debugOn) Log.d(debugTag, "Scaling drive using maxValue: " + maxValue);

            scaleDrive(outputSignal/maxValue, driveValues);  //If max value isn't same as outputSignal, scale so it is
            if (debugOn) Log.d(debugTag, "...Drive Scaled");
            loopEndTime = stopWatch.getElapsedTimeMillis();
            loopDuration = loopEndTime - loopStartTime;
            //Apply drive vector to motors or simulated encoders

            if(simulateMotors){
                if (debugOn) Log.d(debugTag, "Simulating drive step");

                //virForwardEncoder.simulateLoopOutput(outputSignal,loopDuration, currentPose);
                EncoderTracker.updateVirtualEncoders(outputSignal,loopDuration, currentPose);

            } else {
                if (debugOn) Log.d(debugTag, "Applying values to motors");

                // Only assign drive values if error larger than tolerance
                if (currentPose.getErrorMagnitude() > positionalTolerance) {
                    //Now actually assign the calculated drive values to the motors in motorList
                    for (int i = 0; i < motorList.size(); i++) {
                        motorList.get(i).setPower(driveValues[i]);

                        if(debugOn) Log.d(debugTag, motorList.get(i).getPortNumber() + " --> " + format("%2f", driveValues[i]));
                        if(debugOn) Log.d(debugTag, "Num Motors: " + " --> " + motorList.size());
                    }
                } else {
                    stopMotors();
                    if (debugOn) Log.d(debugTag, "Destination reached, motors stopped");
                }
            }

            loopCount++;
            //loopCheckSum = currentPose.getErrorMagnitude() + Math.abs(currentPose.getErrorSum());
            loopMetrics = stopWatch.toString(loopCount);
            writeOdometryTelemetry(loopMetrics, currentPose);
            if (debugOn) Log.d(debugTag, "Error Magnitude: " + currentPose.getErrorMagnitude());
            if (debugOn) Log.d(debugTag, "Position Tolerance: " + positionalTolerance);

            if (debugOn) Log.d(debugTag, "Loop Metrics" + loopMetrics);
            if (debugOn) Log.d(debugTag, "End Position: " + currentPose.toString());
            if (debugOn) Log.d(debugTag, "Target Position: " + currentPose.getTargetPose().toString());

            if (debugOn) Log.d(debugTag, "___________End of Loop_________");

        }

        if(!simulateMotors) stopMotors();

        if (debugOn) Log.d(debugTag, "Forward Encoder: " + forwardTracker.toString());
        if (debugOn) Log.d(debugTag, "Lateral Encoder: " + lateralTracker.toString());

        return currentPose;
    }



    protected void setInitialOffsetForTrackingPose(BNO055IMU imu, TrackingPose trackingPose, Double previousInitialGyroOffset){
        /** This can probably be eliminated
         *
         */
        //If using heading, update the heading using the gyro reading
        Double gyroReading;
        if (useGyroForNavigation) {
            gyroReading = getGyroReadingDegrees();
            trackingPose.setHeadingFromGyro(gyroReading, previousInitialGyroOffset);
        } else{
            gyroReading = trackingPose.getHeading() + previousInitialGyroOffset;
        }
        trackingPose.setInitialGyroOffset(gyroReading);

    }

    protected Double applySoftStartScaling(StopWatch stopWatch){
        /**     This function calculates the Soft Start scaling
         */
        //First calculate he acceleration slope
        Double accelSlope = (outputSignal - minPowerLimit) / softStartDurationMillis;
        Long t = stopWatch.getElapsedTimeMillis();
        Double softStartScale = (accelSlope * t) + minPowerLimit;
        return softStartScale;
    }

    public void correctHeading(TrackingPose currentPose){
        Double currentPoseHeadingError;
        ArrayList<DcMotor> motorList = getDriveMotors();
        String debugTag = "BTI_correctHeading";
        boolean debugOn = true;

        if (debugOn) Log.d(debugTag, "Start of correctHeading, " + currentPose.toString());
        if (debugOn) Log.d(debugTag, "TargetPose, " + currentPose.getTargetPose().toString());
        currentPoseHeadingError = Math.abs(currentPose.getHeading() - currentPose.getTargetPose().getHeading());
        if (debugOn) Log.d(debugTag, "Heading Error: " + currentPoseHeadingError);

        if (currentPoseHeadingError > headingAngleTolerance){
            if (debugOn) Log.d(debugTag, "Starting turn...");

            turnToFieldHeading(currentPose, turnSpeed, motorList);
        }
        //Read new angle from gyro
        currentPose.setHeadingFromGyro(getGyroReadingDegrees());  //Update heading with robot orientation

        //Update the encoder clicks
        EncoderTracker.updateEncoderCurrentClicks();
    }

    protected void prepWebcam(){
        /**
         * Run all the initialization routines for webcam
         */
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }
    @Override
    public void initVuforia() {
        /**
         * Initialize the Vuforia localization engine.
         */

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, WEBCAM_NAME);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    @Override
    public void initTfod() {
        /**
         * Initialize the TensorFlow Object Detection engine.
         */

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void scanForSkystone() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();
            }
        }
    }


    /***********************************************************************
     * Process Post Move activities
     * @param currentPose
     * @param alliance
     ***********************************************************************/
    public void executePostMoveActivity(TrackingPose currentPose, Alliance alliance){
        ArrayList<DcMotor> motorList = getDriveMotors();
        Pose targetPose = currentPose.getTargetPose();
        Pose.PostMoveActivity activity = targetPose.getPostMoveActivity();
        if (activity == Pose.PostMoveActivity.LOWER_RAKE){
            lowerRake();
        } else if (activity == Pose.PostMoveActivity.RAISE_RAKE){
            raiseRake();
        } else if (activity == Pose.PostMoveActivity.SCAN_FOR_SKYSTONE){
            setLifterHeightToGrabStone();
            observeQuarry(currentPose);
            //this is in main loop
            //deliverSkyStonesToBuildingZone(currentPose, motorList, alliance);

        } else if (activity == Pose.PostMoveActivity.RAISE_LIFTER_TO_EXTEND_ARM){
            raiseLifterToExtendArm();
        } else if (activity == Pose.PostMoveActivity.EXTEND_ARM_THEN_LOWER_LIFTER){
            raiseLifterToExtendArm();
            findLifterZero();
        }

    }

    public void lowerRake() {
        //This will also straighten arm and lower lifter

        straightenRotate1();
        lowerLifter();

        foundationRake.setPosition(RAKE_DOWN);
        StopWatch stopWatch = new StopWatch();
        stopWatch.startTimer();
        while (stopWatch.getElapsedTimeMillis() < 1000) {
            //lowering rake
        }
    }

    protected void raiseRake(){
        foundationRake.setPosition(RAKE_UP);
        StopWatch stopWatch = new StopWatch();
        stopWatch.startTimer();
        while(stopWatch.getElapsedTimeMillis()<1000){
            //RAISING rake
        }
    }


    protected void lowerLifter(){
        lifter.setTargetPosition(0);
        lifter.setPower(0.2);
    }


    protected void straightenRotate1(){
        rotate1ClawServo.setPosition(ROTATE1_CLAW_FORWARD);
    }

    protected TrackingPose surveilQuarry(Alliance alliance, TrackingPose currentPose, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_surveyQuarry";

        ArrayList<QuarryStone> observedQuarryStones = new ArrayList<>();
        assignObservedQuarryStones(alliance, observedQuarryStones,1);


        recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                , observedQuarryStones.get(2), alliance);
        if (debugOn) Log.d(logTag, "Quarry Observed " + overallTime.toString());

        TrackingPose endPose = currentPose;
        if(QuarryStone.getCountSkyStones() == 0 && QuarryStone.getCountObserved() <= 1) {
            //If didn't see more than one stone, move right a little and try again
            if (debugOn) Log.d(logTag, "SkyStone not identified, extending search..." + QuarryStone.getCountObserved() +
                    " stone's observed");
            endPose = travelToNextPose(currentPose);
            assignObservedQuarryStones(alliance, observedQuarryStones,1);

            recordQuarryObservations(observedQuarryStones.get(0), observedQuarryStones.get(1)
                    , observedQuarryStones.get(2), alliance);
        }

        //  TODO:  Improve this logic for inclusion of second vantage point
        determineSkyStonePattern(observedQuarryStones.get(0), observedQuarryStones.get(1));
        return endPose;
    }

    protected void assignObservedQuarryStones(Alliance alliance, ArrayList<QuarryStone> observedQuarryStones, int vantagePoint){
        QuarryStone firstStone;
        QuarryStone secondStone;
        QuarryStone thirdStone;

        if (alliance == Alliance.BLUE && vantagePoint == 1) {
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.TWO);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.THREE);
            thirdStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.FOUR);
        } else if (alliance == Alliance.BLUE && vantagePoint == 2) {
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.ONE);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.TWO);
            thirdStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.THREE);
        } else if (alliance == Alliance.RED && vantagePoint == 1) {
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.THREE);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.FOUR);
            thirdStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.FIVE);
        } else if (alliance == Alliance.RED && vantagePoint == 2) {
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.TWO);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.THREE);
            thirdStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.FOUR);
        } else {
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.ONE);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.TWO);
            thirdStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.THREE);
        }

        if (observedQuarryStones.size() > 0) observedQuarryStones.clear();
        observedQuarryStones.add(firstStone);
        observedQuarryStones.add(secondStone);
        observedQuarryStones.add(thirdStone);
    }

    protected void observeQuarry(TrackingPose currentPose) {
        //Based on the Pose x coordinate, determine which stones are being viewed
        QuarryStone firstStone;
        QuarryStone secondStone;
        if (currentPose.getX() < -55.0) {
            //Near the end of the field
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.ZERO);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.ONE);
        } else if (currentPose.getX() < -39.0) {
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.TWO);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.THREE);
        } else {
            firstStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.FOUR);
            secondStone = QuarryStone.getQuarryStone(QuarryStone.StoneLocation.FIVE);
        }
        recordQuarryObservations(firstStone, secondStone);
        determineSkyStonePattern(firstStone, secondStone);

    }


    /**
     * Method is intended to be a short pause to allow the vuforia object detector to record observations
     * related to the Quarry Stones
     * @param firstStone
     * @param secondStone
     */
    protected void recordQuarryObservations(QuarryStone firstStone, QuarryStone secondStone){
        int numObservations = 0;
        long timeout = 2000L;
        StopWatch timer = new StopWatch();

        if (tfod != null) {
            while (opModeIsActive() && timer.getElapsedTimeMillis() < timeout) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData("Num Observations", numObservations);
                        // Determine which stone to update based on the position in the frame
                        QuarryStone currentStone;
                        if (recognition.getLeft() < 150) {
                            currentStone = secondStone;
                        } else currentStone = firstStone;

                        //  See if it is recorded as skyStone
                        Boolean isSkyStone;
                        if (recognition.getLabel().equalsIgnoreCase("SkyStone")) {
                            isSkyStone = true;
                        } else isSkyStone = false;

                        currentStone.recordObservation(isSkyStone);

                    }
                    telemetry.update();
                }
                numObservations++;
            }
        }
    }

    protected void recordQuarryObservations(QuarryStone firstStone, QuarryStone secondStone, QuarryStone thirdStone, Alliance alliance){
        int numObservations = 0;
        long timeout = 2000L;
        StopWatch timer = new StopWatch();

        if (tfod != null) {
            while (opModeIsActive() && timer.getElapsedTimeMillis() < timeout) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData("Num Observations", numObservations);
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

                        currentStone.recordObservation(isSkyStone);

                    }
                    telemetry.update();
                }
                numObservations++;
            }
        }
    }


    protected void determineSkyStonePattern(QuarryStone firstStone, QuarryStone secondStone){
        /**
         * Based on which stone is a SkyStone, generate the arraylist of stones
         */
        String logTag = "BTI_determineSkyStone.";
        Log.d(logTag, "Determining Stone Pattern...");
        QuarryStone.StoneLocation observedSkyStoneLocation;
        //Based on which position the observed skyStone was, the pa
        if (firstStone.isSkyStone()){
            Log.d(logTag, "First stone is skystone " + firstStone.toString());
            observedSkyStoneLocation = firstStone.getStoneLocation();
        } else if (secondStone.isSkyStone()){
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


    protected QuarryStone.StoneLocation inferThirdPosition(QuarryStone firstStone){
        QuarryStone.StoneLocation inferredLocation;
        if(firstStone.getStoneLocation() == QuarryStone.StoneLocation.ZERO){
            inferredLocation = QuarryStone.StoneLocation.TWO;
        } else if (firstStone.getStoneLocation() == QuarryStone.StoneLocation.ONE){
            inferredLocation = QuarryStone.StoneLocation.THREE;
        } else if (firstStone.getStoneLocation() == QuarryStone.StoneLocation.TWO){
            inferredLocation = QuarryStone.StoneLocation.FOUR;
        } else if (firstStone.getStoneLocation() == QuarryStone.StoneLocation.THREE){
            inferredLocation = QuarryStone.StoneLocation.FIVE;
        } else if (firstStone.getStoneLocation() == QuarryStone.StoneLocation.FOUR){
            inferredLocation = QuarryStone.StoneLocation.ZERO;
        } else inferredLocation = QuarryStone.StoneLocation.FIVE;
        return inferredLocation;
    }

    protected QuarryStone getTargetSkyStone(){
        /**
         * Returns a skyStone for transport to Building Zone
         * If none were observed, it will pick a random one
         */

        String logTag = "BTI_getTargetSkyStone";
        //Pick up the SkyStone
        Log.d(logTag, "Retrieving skyStone coordinates, " + QuarryStone.getFoundSkyStoneCount()
                + " skystones found");

        ArrayList<QuarryStone> skyStones = QuarryStone.getSkyStones();
        ListIterator<QuarryStone> quarryStoneListIterator = skyStones.listIterator();
        QuarryStone skyStone;
        QuarryStone.StoneLocation skyStoneLocation;

        //If skystone is found, then retrieve the first item on the list,
        //otherwise pick a random number
        if (quarryStoneListIterator.hasNext()){
            skyStone = quarryStoneListIterator.next();
            skyStoneLocation = skyStone.getStoneLocation();
            Log.d(logTag, "Going for stone: " + skyStone.toString());
            Log.d(logTag, "Removing from list, currently with " + skyStones.size() + " items");

            //Once the location is known, remove it from the list
            quarryStoneListIterator.remove();
            Log.d(logTag, "Item removed, now with " + skyStones.size() + " items");
        } else {
            int randomNum = ThreadLocalRandom.current().nextInt(1, 6);  //note, excludes block against wall
            skyStoneLocation = QuarryStone.StoneLocation.getStoneLocation(randomNum);
            skyStone = QuarryStone.getQuarryStone(skyStoneLocation);
            Log.d(logTag, "Going for random stone: " + skyStone.toString());
        }
        return skyStone;
    }

    protected TrackingPose possessSkyStone(TrackingPose endPose, Alliance alliance, StopWatch overallTime){
        //  grab a skystone
        boolean debugOn = true;
        String logTag = "BTI_possessSkyStone";
        endPose = driveToSkyStone(endPose, alliance);
        if (debugOn) Log.d(logTag, "....COMPLETED Drive to Skystone " + overallTime.toString());
        if (debugOn) Log.d(logTag, "endPose Position:" + endPose.toString());

        if (debugOn) Log.d(logTag, "Refining position to stone...");
        endPose = refinePosition(FieldObject.QUARRY_STONE, endPose, overallTime);

        if (debugOn) Log.d(logTag, "...COMPLETED refining position to stone " + overallTime.toString());
        autoGrabBlockNoMovement(getDriveMotors());
        return endPose;
    }

    protected TrackingPose moveBackToClearQuarry(TrackingPose endPose, Alliance alliance, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_moveBackToClearQ";

        if (debugOn) Log.d(logTag, "Moving back to clear Quarry...");

        //**  Move back a little to clear quarry
        double signYCoord = (alliance == Alliance.BLUE) ? 1.0 : -1.0;
        double backtrackDist = 10.0 * signYCoord;
        Pose startPose = new Pose (endPose.getX(), endPose.getY(), endPose.getHeading());

        //Modified in V2 to execute the turn prior to driving to quarry
        Pose targetPose = new Pose (endPose.getX(), endPose.getY() + backtrackDist, 0.0);
        TrackingPose currentPose = new TrackingPose(startPose, targetPose, endPose.getInitialGyroOffset());
        endPose = travelToNextPose(currentPose);
        if (debugOn) Log.d(logTag, "...COMPLETED MOVE BACK " + overallTime.toString());
        if (debugOn) Log.d(logTag, endPose.toString());

        //Execute the turn towards back wall defined above
        correctHeading(endPose);

        return endPose;
    }

    protected TrackingPose travelUnderBridgeToFoundation(TrackingPose endPose, Alliance alliance, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_travelUnderBridge";

        //Travel under bridge toward foundation
        //But need to lift arm first, so to to  PoseAfterPlaceSkystone
        if (debugOn) Log.d(logTag, "Getting tracking pose across bridge ");
        TrackingPose currentPose = getTrackingPoseAcrossBridge(endPose);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge ");
        endPose = travelToNextPose(currentPose);     //Actually drive
        if (debugOn) Log.d(logTag, "Travel across bridge completed, Location: " + endPose.toString());
        if (debugOn) Log.d(logTag, "overallTime: "  + overallTime.toString());

        return endPose;
    }
    protected TrackingPose deliverSkyStonesToBuildingZone(TrackingPose endPose, ArrayList<DcMotor> motorList, Alliance alliance){
        /**
         * Drive from current location to the next SkyStone
         */

        String logTag = "BTI_deliverSkyStone...";

        //Move the arm to correct height to grab stone
        setLifterHeightToGrabStone();

        //Retrieve a skystone from those observed.  If not observed, select random
        QuarryStone skyStone = getTargetSkyStone();

        //Travel to the SkyStone
        Log.d(logTag, "Getting tracking pose to skystone");
        TrackingPose currentPose = getTrackingPoseToSkyStone(skyStone, endPose, alliance);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving to skyStone");
        endPose = travelToNextPose(currentPose);     //Actually drive
        Log.d(logTag, "SkyStone travel completed, Location: " + currentPose.toString());

        //**  Correct Heading
        correctHeading(endPose);
        //**  Now perform the autoGrab function
        autoGrabBlockNoMovement(motorList);

        //**  Move back a little to clear quarry
        double signYCoord = (alliance == Alliance.BLUE) ? 1.0 : -1.0;
        double backtrackDist = 5.0 * signYCoord;
        Pose startPose = new Pose (endPose.getX(), endPose.getY(), endPose.getHeading());
        Pose targetPose = new Pose (endPose.getX(), endPose.getY() + backtrackDist, endPose.getHeading());
        currentPose = new TrackingPose(startPose, targetPose, endPose.getInitialGyroOffset());
        endPose = travelToNextPose(currentPose);


        //Travel under bridge to foundation
        Log.d(logTag, "Getting tracking pose across bridge");
        currentPose = getTrackingPoseAcrossBridge(endPose);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge");
        endPose = travelToNextPose(currentPose);     //Actually drive
        Log.d(logTag, "Travel across bridge completed, Location: " + currentPose.toString());

        //Now spin toward Foundation to dump stone
        setLifterHeightToPlaceStone();  //raise lifter to place on foundation

        //**  Correct Heading
        correctHeading(endPose);

        //  Move forward a little bit more to dump block on foundation
        Log.d(logTag, "Getting tracking pose to foundation");
        currentPose = getTrackingPoseToFoundation(endPose);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving to foundation");
        endPose = travelToNextPose(currentPose);     //Actually drive
        Log.d(logTag, "Foundation travel completed, Location: " + currentPose.toString());

        //**  Correct Heading
        correctHeading(endPose);

        //  Dump stone
        autoReleaseStone(Speed.SLOW);


        //  move back a little to clear foundation plate
        Log.d(logTag, "Backing away from foundation plate");
        currentPose = getTrackingPoseAcrossBridge(endPose);  //Set course
        Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge");
        endPose = travelToNextPose(currentPose);     //Actually drive
        Log.d(logTag, "Backed away, ready to drop lifter: " + currentPose.toString());

        //drop lifter
        findLifterZero();

        return endPose;
    }

    protected TrackingPose backAwayFromFoundation(TrackingPose endPose, Alliance alliance, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_backAwayFromF";

        if (debugOn) Log.d(logTag, "Backing away from foundation plate");
        TrackingPose currentPose = getTrackingPoseAfterPlaceSkystone(endPose, alliance);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving across bridge");
        endPose = travelToNextPose(currentPose);     //Actually drive
        if (debugOn) Log.d(logTag, "Backed away, ready to drop lifter: " + currentPose.toString());
        if (debugOn) Log.d(logTag, "overallTime: "  + overallTime.toString());

        return endPose;
    }

    protected TrackingPose driveToSkyStone(TrackingPose endPose, Alliance alliance){
        /**
         * Drive from current location to the next SkyStone
         */

        String logTag = "BTI_driveToSkyStone";
        boolean debugOn = true;
        if (debugOn) Log.d(logTag, "starting driveToSkyStone, current Position " + endPose.toString());

        ArrayList<DcMotor> driveMotors= getDriveMotors();

        //Retrieve a skystone from those observed.  If not observed, select random
        QuarryStone skyStone = getTargetSkyStone();

        //Travel to the SkyStone
        if (debugOn) Log.d(logTag, "Getting tracking pose to skystone");
        TrackingPose currentPose = getTrackingPoseToSkyStone(skyStone, endPose, alliance);  //Set course
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~Driving to skyStone at " + currentPose.getTargetPose().toString());
        endPose = travelToNextPose(currentPose);     //Actually drive
        if (debugOn) Log.d(logTag, "SkyStone travel completed, Location: " + endPose.toString());

        //**  Correct Heading
        correctHeading(endPose);

        return endPose;
    }

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

        }
        String resultString = (isTimedOut) ? "FAILURE - Timed out" : "SUCCESS";
        //  This updates encoders without calculating field position, maybe not correct
        //  Perhaps should use getNewPose instead
        //EncoderTracker.updateEncoderCurrentClicks();
        //  Alternate would be this
        EncoderTracker.getNewPose(trackingPose);

        if (debugOn) Log.d(logTag, "COMPLETED Refining position to " + fieldObject.name() +
                " Result: " + resultString);
        if (debugOn) Log.d(logTag, "Overall Time: " + overallTime.toString());

        return trackingPose;
    }


    protected void applyRedAlliancePoseTransform(Pose pose){
        //  Hard-coded poses are valid for the Blue side
        //  Apply the transformation to make applicable to Red side
        //  -->  Change sign of Y position
        //  -->  Change sign of Heading
        pose.setY(-pose.getY());              //  -->  Change sign of Y position
        pose.setHeading(-pose.getHeading());  //The called method checks heading bounds (so -180 will become 180)
    }
    protected TrackingPose getTrackingPoseToSkyStone(QuarryStone skyStone, TrackingPose endPose, Alliance alliance){
        String logTag = "BTI_getTrack.SkyStone.";
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

        //Create the beginning and end poses for the move
        Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        Pose skyStonePose = new Pose(stoneX,   stoneY , stoneHeading);
        Log.d(logTag, "Creating tracking pose to SkyStone " + skyStone.toString());
        //Create the tracking pose
        return new TrackingPose(startPose, skyStonePose, endPose.getInitialGyroOffset());
    }

    protected TrackingPose getTrackingPoseAcrossBridge(TrackingPose endPose){
        String logTag = "BTI_getTracking.Bridge.";
        Log.d(logTag, "Creating tracking pose to en route to Foundation under bridge...");

        //Create the beginning and end poses for the move

        Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        Pose acrossBridge = foundation.getSkyStoneDumpingPose();
        Log.d(logTag, "Creating tracking pose across bridge COMPLETED!");
        return new TrackingPose(startPose, acrossBridge,endPose.getInitialGyroOffset());
    }


    protected TrackingPose getTrackingPoseAfterPlaceSkystone(TrackingPose endPose, Alliance alliance){
        String logTag = "BTI_getTracking.Place.";
        Log.d(logTag, "Creating tracking pose after dropping skyStone...");

        //Create the beginning and end poses for the move

        Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        Pose afterPlaceSkyStone = foundation.getPoseAfterPlaceSkystone(endPose, alliance);
        Log.d(logTag, "Creating tracking pose after place SkyStone COMPLETED!");
        return new TrackingPose(startPose, afterPlaceSkyStone,endPose.getInitialGyroOffset());
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

    protected TrackingPose parkUnderBridge(TrackingPose endPose, Alliance alliance, StopWatch overallTime){
        boolean debugOn = true;
        String logTag = "BTI_parkUnderBridge";
        TrackingPose currentPose = getTrackingPoseToBridgePark(endPose, alliance);
        if (debugOn) Log.d(logTag, "~~~~~~~~~~~~~parking under bridge");
        endPose = travelToNextPose(currentPose);     //Actually drive
        if (debugOn) Log.d(logTag, "...Parked under bridge " + endPose.toString());
        if (debugOn) Log.d(logTag, "Time " + overallTime.toString());

        return endPose;
    }

    protected TrackingPose getTrackingPoseToBridgePark(TrackingPose endPose, Alliance alliance){
        String logTag = "BTI_getTracking.Park.";
        Log.d(logTag, "Creating tracking pose to park under bridge...");

        //Create the beginning and end poses for the move

        Pose startPose = new Pose(endPose.getX(), endPose.getY(), endPose.getHeading());
        Pose bridgePark = Pose.getBridgeParkPose(endPose, alliance);
        Log.d(logTag, "Creating tracking pose to bridge park COMPLETED!");
        return new TrackingPose(startPose, bridgePark,endPose.getInitialGyroOffset());
    }


    protected void autoGrabBlockNoMovement(ArrayList<DcMotor> motorList){
        /**
         * Automated routine to grab a block
         * 1) raise the lifter
         * 2) lower lifter while moving grabber wheel
         * 3) stop wheel and lift for travel
         */

        //  1) raise the lifter
        setLifterHeightToGrabStone();

        //  2) Grab Stone (also lifts a little for travel
        autoGrabStone(motorList);

    }

}
