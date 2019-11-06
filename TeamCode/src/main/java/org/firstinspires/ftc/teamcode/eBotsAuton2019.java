package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

import static java.lang.String.format;

public abstract class eBotsAuton2019 extends eBotsOpMode2019 {

    /*****************************************************************
    //******    CONFIGURATION PARAMETERS
    //****************************************************************/

    protected Boolean simulateMotors = false;
    protected Boolean useGyroForNavigation;
    protected Integer gyroCallFrequency;
    protected Double saturationLimit;
    protected Double turnSpeed;
    protected double pGain;
    protected double iGain;


    protected Boolean useSoftStart;
    protected Long softStartDurationMillis;
    protected Double minPowerLimit;

    protected Double headingAngleTolerance;
    protected Double positionalTolerance;
    protected Double integratorUnwindLimit;

    /*****************************************************************
    //******    CLASS VARIABLES
    //****************************************************************/

    protected Double pSignal = 0.0;
    protected Double iSignal = 0.0;
    protected Double computedSignal=0.0;
    protected Double outputSignal=0.0;

    protected EncoderTracker forwardTracker;
    protected EncoderTracker lateralTracker;
    protected ArrayList<Pose> wayPoses;
    /****************************************************************/


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
    }

    public enum Speed{
        SLOW (0.35, 0.3, 0.15, 0.0),
        MEDIUM (0.60,0.3,  0.15, 0.0),
        FAST (0.8, 0.3, 0.05, 0.0);

        /**  ENUM VARIABLES     **************/
        private Double maxSpeed;
        private Double turnSpeed;
        private Double pGainEnum;
        private Double iGainEnum;

        /**  CONSTRUCTOR    **************/
        Speed(Double speed, Double turnSpeed, Double pGain, Double iGain){
            maxSpeed = speed;
            turnSpeed = turnSpeed;
            pGainEnum = pGain;
            iGainEnum = iGain;
        }
        /**  ENUM GETTERS AND SETTERS  ***********/
        public Double getMaxSpeed(){return this.maxSpeed;}
        public Double getTurnSpeed(){return this.turnSpeed;}
        public Double getpGainEnum(){return this.pGainEnum;}
        public Double getiGainEnum(){return this.iGainEnum;}
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
            gyroOn = gyroPower;
            readFrequency = frequency;
        }

        /**  ENUM GETTERS AND SETTERS  ***********/
        public Boolean isGyroOn(){return this.gyroOn;}
        public Integer getReadFrequency(){return this.readFrequency;}
    }

    public enum SoftStart{
        NO (false, 0L, 0.2),
        MEDIUM (true, 750L, 0.2),
        SLOW_START(true, 1500L, 0.2);

        /**  ENUM VARIABLES     **************/
        private Boolean softStartOn;
        private Long durationMillis = 750L;
        private Double minPower = 0.20;

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
        STANDARD (5.0, 0.2, 100.0),
        TIGHT (2.5, 0.1, 25.0);

        /**  ENUM VARIABLES     **************/

        private Double headingAngleAccuracy;
        private Double positionalAccuracy;
        private Double integratorUnwindLimit;

        /**  CONSTRUCTOR    **************/

        Accuracy(Double headingAngleAccuracy, Double positionalAccuracy, Double integratorUnwind){
            this.headingAngleAccuracy = headingAngleAccuracy;
            this.positionalAccuracy = positionalAccuracy;
            this.integratorUnwindLimit = integratorUnwind;
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
    }

    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/


    /*****************************************************************
     //******    CLASS METHODS
     //****************************************************************/

    @Override
    protected void setWayPoses(ArrayList<Pose> wayPoses, Alliance alliance, FieldSide fieldSide) {
        /**  This function sets the wayPoses for blue side and then applies a
         * transformation by mirroring about the X axis for position and pose
         */

        if (wayPoses.size() > 0) wayPoses.clear();       // get rid of pre-existing poses

        //These waypoints are for the blue side
        if (fieldSide == FieldSide.FOUNDATION){
            //Blue Foundation
            wayPoses.add(new Pose(39.0, 60.0, 90.0));
            //travel in the positive Y direction to midline
            wayPoses.add(new Pose(50.0, 34.0, 90.0, Pose.PostMoveActivity.LOWER_RAKE));
            //travel back 12 inches
            wayPoses.add(new Pose(54.0, 56.0, 90.0, Pose.PostMoveActivity.RAISE_RAKE));
            //travel to blue foundation start point
            wayPoses.add(new Pose(22.0, 56.0, 90.0));
            //travel back 12 inches
            wayPoses.add(new Pose(22.0, 16.0, 90.0));
            //travel to blue foundation start point
            wayPoses.add(new Pose(48.0, 16.0, 90.0));
            //travel to blue foundation start point
            wayPoses.add(new Pose(48.0, 40.0, 90.0));
            //move back a little
            wayPoses.add(new Pose(48.0, 35.0, 90.0));
            //move around foundation
            wayPoses.add(new Pose(18.0, 35.0, 90.0));
            //move near wall
            wayPoses.add(new Pose(18.0, 60.0, 90.0));
            //park in middle
            wayPoses.add(new Pose(0.0, 60.0, 90.0));

            //Park in the middle
            //wayPoses.add(new Pose(-48.0, 0.0, 180.0));
        } else if (fieldSide == FieldSide.QUARRY){
            wayPoses.add(new Pose(Pose.StartingPose.BLUE_QUARRY));
            //travel in the positive Y direction to midline
            wayPoses.add(new Pose(-12.0, 50.0, -90.0));
            //travel back 12 inches
            wayPoses.add(new Pose(-52.0, 50.0, -90.0));
            //travel to blue foundation start point
            wayPoses.add(new Pose(-52.0, 60.0, -90.0));
            //park in middle
            wayPoses.add(new Pose(0.0, 60.0, -90.0));

        } else {
            //start at the origin facing X axis
            wayPoses.add(new Pose(0.0,0.0,90.0));
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
        if (alliance == Alliance.RED){
            for (Pose p:wayPoses){
                p.setY(-p.getY());      //Flip sign for X
                p.setHeading(-p.getHeading());  //The called method checks heading bounds (so -180 will become 180)
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
        pGain = speedConfig.getpGainEnum();
        iGain = speedConfig.getiGainEnum();
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

    protected void writeOdometryTelemetry(String loopMetrics, TrackingPose currentPose, EncoderTracker encoder1, EncoderTracker encoder2) {
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

    protected void initializeEncoderTrackers(){
        //Initialize virtual encoders
        EncoderTracker.purgeExistingEncoderTrackers();
        Log.d("initEncoderTrackersVir", "Initializing Virtual Encoders");
        VirtualEncoder virForwardEncoder = new VirtualEncoder();  //if using virtual
        VirtualEncoder virLateralEncoder = new VirtualEncoder();  //if using virtual
        forwardTracker = new EncoderTracker(virForwardEncoder, EncoderTracker.RobotOrientation.FORWARD);
        lateralTracker = new EncoderTracker(virLateralEncoder, EncoderTracker.RobotOrientation.LATERAL);
    }

    protected void initializeEncoderTrackers(ArrayList<DcMotor> motorList){
        //Initialize actual encoders

        EncoderTracker.purgeExistingEncoderTrackers();      //Clean out any pre-existing encoders

        Log.d("initEncoderTrackersReal", "Initializing Real Encoders");
        forwardTracker = new EncoderTracker(motorList.get(3), EncoderTracker.RobotOrientation.FORWARD);
        lateralTracker = new EncoderTracker(motorList.get(1), EncoderTracker.RobotOrientation.LATERAL);
    }


    protected TrackingPose travelToNextPose(TrackingPose currentPose, ArrayList<DcMotor> motorList){
        Integer loopCount = 0;
        String loopMetrics;
        Double previousSignalSign=0.0;
        Boolean isSaturated = true;
        Boolean isLowPower = false;
        Boolean softStartActive = true;
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
        while(opModeIsActive() && currentPose.getErrorMagnitude() > positionalTolerance){
            //while(opModeIsActive() && loopCount<299){
            Log.d("travelToNextPose", "*******" + loopCount.toString());
            Log.d("travelToNextPose", "numEncoders" + EncoderTracker.getEncoderTrackerCount());

            double[] driveValues = new double[4];               //Array of values for the motor power levels
            double maxValue;

            loopStartTime = loopEndTime;        //Uses the end time of the previous loop for start time (ensures continuity of cycles)

            if (loopCount>0){  //For every iteration after the first
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

            currentPose.setHeadingErrorLocked(isSaturated, driveSignalSignChange);
            currentPose.updateTravelDirection();

            //Apply bounds to the output signal:
            //      ** Clip high end at saturationLimit
            //      ** Clip low end at minPowerLimit
            //      ** If within transient period, ramp down the power
            outputSignal = (isSaturated) ? saturationLimit : Math.abs(computedSignal);  //Note: always positive unlike computed signal
            outputSignal = (isLowPower)  ? minPowerLimit   : outputSignal;
            if (softStartActive) outputSignal = applySoftStartScaling(stopWatch);

            Log.d("BTI_ruOpMode", printDriveSignalMetrics() + "--" + signChangeDebug);

            calculateFieldOrientedDriveVector(currentPose.getTravelDirectionRad(), currentPose.getHeadingRad(),outputSignal,0,driveValues);
            //Scale the drive vector to so the max motor speed equals the outputSignal value
            //The calculation can generate values less than outputSignal
            //This scaling provides better speed control
            maxValue = findMaxAbsValue(driveValues);  //See what the max drive value is set to
            scaleDrive(outputSignal/maxValue, driveValues);  //If max value isn't same as outputSignal, scale so it is

            loopEndTime = stopWatch.getElapsedTimeMillis();
            loopDuration = loopEndTime - loopStartTime;
            //Apply drive vector to motors or simulated encoders

            if(simulateMotors){
                //virForwardEncoder.simulateLoopOutput(outputSignal,loopDuration, currentPose);
                EncoderTracker.updateVirtualEncoders(outputSignal,loopDuration, currentPose);

            } else {
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

        if(!simulateMotors) stopMotors(motorList);

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

    public void correctHeading(TrackingPose currentPose, ArrayList<DcMotor> motorList){
        Double currentPoseHeadingError;
        currentPoseHeadingError = Math.abs(currentPose.getHeading() - currentPose.getTargetPose().getHeading());
        if (currentPoseHeadingError > headingAngleTolerance){
            turnToFieldHeading(currentPose, turnSpeed, motorList);
        }
        //Read new angle from gyro
        currentPose.setHeadingFromGyro(getGyroReadingDegrees());  //Update heading with robot orientation


    }

    public void executePostMoveActivity(TrackingPose currentPose){
        Pose targetPose = currentPose.getTargetPose();
        Pose.PostMoveActivity activity = targetPose.getPostMoveActivity();
        if (activity == Pose.PostMoveActivity.LOWER_RAKE){
            lowerRake();
        } else if (activity == Pose.PostMoveActivity.RAISE_RAKE){
            raiseRake();
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

    protected void raiseLifter(){
        lifter.setTargetPosition(-2000);
        lifter.setPower(0.2);
        StopWatch stopWatch = new StopWatch();
        stopWatch.startTimer();
        while(stopWatch.getElapsedTimeMillis()<1000){
            //RAISING lifter
        }
    }

    protected void lowerLifter(){
        lifter.setTargetPosition(0);
        lifter.setPower(0.2);
    }


    protected void straightenRotate1(){
        rotate1ClawServo.setPosition(ROTATE1_CLAW_FORWARD);
    }

}
