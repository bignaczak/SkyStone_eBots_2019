package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.ListIterator;

import static java.lang.String.format;

public class EncoderTracker {
    /**  EncoderTracker records and computes the differences in encoder positions
    //  and translates them back into x,y (and maybe heading eventually)
    */

    /***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88*/

    private DcMotor motor;                      //motor that the encoder is attached to
    private RobotOrientation robotOrientation;  //orientation of the encoder relative to robot reference frame
                                                //  FORWARD
                                                //  LATERAL -> Left is positive

    private Integer currentClicks;              //  Number of clicks for current iteration
    private Integer cumulativeClicks;                //  Cumulative Number of Clicks
    private Double cumulativeDistance;               //  Total Distance traveled in inches
    private VirtualEncoder virtualEncoder;
    private double wheelDiameter;
    private double spinRadius;
    private SpinBehavior spinBehavior;          //
    private ClickDirection clickDirection;

    /***************************************************************88
    //******    STATIC VARIABLES
    //****************************************************************/
    private static Double clicksPerInch = 434.6;   //  Clicks per inch of travel

    //A list of all the encoders being tracked
    private static ArrayList<EncoderTracker> encoders = new ArrayList<>();

    /***************************************************************88
    //******    ENUMERATIONS
    //****************************************************************/

    public enum RobotOrientation{
        FORWARD,
        LATERAL,
    }

    public enum SpinBehavior{
        /**
         * Describes whether the encoder count increases or decreases with positive angle change
         */
        INCREASES_WITH_ANGLE
        , DECREASES_WITH_ANGLE
    }

    public enum ClickDirection{
        /**
         * Describes whether the click count increases or decreases with positive x/y travel (forward/left travel)
         */
        STANDARD,
        REVERSE
    }

    public enum FieldDirection{
        X, Y
    }

    /****************************************************************
    //******    CONSTRUCTORS
    //***************************************************************/

    public EncoderTracker (DcMotor motor, RobotOrientation robotOrientation){
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.robotOrientation = robotOrientation;
        this.currentClicks = this.getClicks();
        this.cumulativeDistance = 0.0;
        this.cumulativeClicks =0;
        this.virtualEncoder = null;
        this.wheelDiameter = 3.0;
        this.spinRadius = 6.0;
        this.spinBehavior = SpinBehavior.INCREASES_WITH_ANGLE;
        this.clickDirection = ClickDirection.STANDARD;
        encoders.add(this);
    }

    public EncoderTracker (VirtualEncoder virtualEncoder, RobotOrientation robotOrientation){
        this.motor = null;
        this.virtualEncoder = virtualEncoder;
        this.robotOrientation = robotOrientation;
        this.currentClicks = this.getClicks();
        this.cumulativeDistance = 0.0;
        this.cumulativeClicks =0;
        this.wheelDiameter = 3.0;
        this.spinRadius = 8.0;
        this.spinBehavior = SpinBehavior.INCREASES_WITH_ANGLE;
        this.clickDirection = ClickDirection.STANDARD;
        encoders.add(this);
    }

    /***************************************************************
    //******    STATIC METHODS
    //****************************************************************/

    public static void getNewPose (TrackingPose trackingPose){
        boolean debugOn = false;
        String debugTag = "BTI_getNewPose";
        int incrementalClicks;                   //new encoder position for this iteration
        Double newX = trackingPose.getX();   //starting position for x
        Double newY = trackingPose.getY();   //starting position for y
        Double encoderAngle;                 //to track field-oriented encoder orientation
        Double totalDistance;                //linear distance traveled before x and y components calculated

        if (debugOn) {
            Log.d(debugTag, "Start:" + trackingPose.toString());
            Log.d(debugTag, "Error: " + trackingPose.printError());
            Log.d(debugTag, "Num Encoders: " + encoders.size());
        }
        // TODO: 10/13/2019 Add consideration for rotation
        ListIterator<EncoderTracker> listIterator = encoders.listIterator();
        while(listIterator.hasNext()){
        //  Loop through the encoders to figure out how much
        //for (EncoderTracker e: encoders){
            //Calculate distance traveled for encoder
            EncoderTracker e = listIterator.next();
            if (debugOn) {
                Log.d(debugTag, e.toString());
            }
            int newReading = e.getClicks();
            incrementalClicks = newReading - e.currentClicks;
            totalDistance = Math.abs((incrementalClicks) / clicksPerInch);

            e.cumulativeClicks += Math.abs(incrementalClicks);
            e.cumulativeDistance += Math.abs(totalDistance);

            //Consider the placement of the encoder on the robot
            //This assumes that when robot front oriented with positive X direction
            //  that clicks increase with x+ and y+ field oriented vectors
            encoderAngle = trackingPose.getHeading();

            //Lateral encoder must shift angle to calculate effect on X and Y coordinates
            if(e.robotOrientation == RobotOrientation.LATERAL) {
                encoderAngle += 90;
            }

            //  Relate the distance traveled to field-oriented x and y translation
            Double deltaX = totalDistance * Math.cos(Math.toRadians(encoderAngle));
            Double deltaY = totalDistance * Math.sin(Math.toRadians(encoderAngle));

            //  Apply sign to change in values based on travelAngle
            //  X is increasing is travelDirection is -90 < travelDirection <90
            if (Math.abs(trackingPose.getTravelDirection()) < 90){
                deltaX = Math.abs(deltaX);
            } else{
                deltaX = -Math.abs(deltaX);
            }
            //  Y is increasing if travelDirection is greater than 0 and less than or equal to 180
            //  Remember, 0 is considered negative (because 180 is positive)
            if (trackingPose.getTravelDirection() > 0 && trackingPose.getTravelDirection() <= 180){
                deltaY = Math.abs(deltaY);
            } else{
                deltaY = -Math.abs(deltaY);
            }

            //  Set the new positions by adding deltas to corresponding coordinate
            //  These are cumulative for each encoder analyzed
            newX += deltaX;
            newY += deltaY;

            if (debugOn) Log.d(debugTag + " D/x/y", e.robotOrientation.toString() + " " +
                    format("%.3f", totalDistance) + " / " +
                    format("%.3f", deltaX) + " / " + format("%.3f", deltaY));

            //  Update the encoder position for next iteration of the loop
            e.currentClicks = newReading;
        }
        if (debugOn) Log.d(debugTag, "About to update position with new X and Y coordinates");
        trackingPose.setX(newX);
        trackingPose.setY(newY);
        if (debugOn) Log.d(debugTag, "...Completed");
        //Log.d(debugTag + " Ending", trackingPose.toString());

        //trackingPose.setHeading();        //Add this once the 3rd encoder is installed
    }

    public static void updatePoseUsingThreeEncoders(TrackingPose trackingPose, BNO055IMU imu){
        /**
         * Update tracking pose based on the readings of 3 encoders
         * One direction of travel has double Encoders to capture spin
         */
        boolean debugOn = true;
        String logTag = "BTI_updatePose...Three";
        if (debugOn) {
            Log.d(logTag, "Entering updatePoseUsingThreeEncoders");
            Log.d(logTag, "Starting encoder positions");
            for(EncoderTracker e: encoders){
                e.toString();
            }
        }
        RobotOrientation doubleEncoderDirection = RobotOrientation.FORWARD;

        ArrayList<EncoderTracker> doubleEncoders = getDoubleEncoders(doubleEncoderDirection);
        EncoderTracker singleEncoder = getSingleEncoder(doubleEncoderDirection);
        if (singleEncoder == null) {
            if (debugOn) Log.d(logTag, "Error finding single encoder");
            return;
        }

        // Capture spin and translate info from double Encoders
        int spinClicks = calculateSpinClicks(doubleEncoders);
        int translateClicks = calculateTranslateClicks(doubleEncoders);

        // Capture single encoder incremental clicks and then apply
        // spin information to discern between translation and rotation
        int singleEncoderClicks = singleEncoder.getIncrementalClicks();

        //Because the single encoder is a different distance from the centerpoint, a different spinClicks must be calculated
        double deltaAngle = calculateSpinAngle(spinClicks, doubleEncoders);
        int singleEncoderSpinClicks = calculateSingleEncoderSpinClicks(deltaAngle, singleEncoder.getSpinRadius());
        int singleTranslateClicks = singleEncoderClicks - singleEncoderSpinClicks;


        //TODO:  Consider either performing this earlier or temporarily storing read values
        //After reading in values, update encoders currentClicks variable with the present encoder values
        updateEncoderCurrentClicks();


        double deltaX = 0.0;
        double deltaY = 0.0;
        for (RobotOrientation ro: RobotOrientation.values()){
            if(ro == doubleEncoderDirection){
                deltaX += calculateTranslateComponent(translateClicks, FieldDirection.X, ro, trackingPose);
                deltaY += calculateTranslateComponent(translateClicks, FieldDirection.Y, ro, trackingPose);
            } else {
                deltaX += calculateTranslateComponent(singleTranslateClicks, FieldDirection.X, ro, trackingPose);
                deltaY += calculateTranslateComponent(singleTranslateClicks, FieldDirection.Y, ro, trackingPose);
            }
        }


        //Now update the tracking pose with the new location and heading
        trackingPose.setX(trackingPose.getX() + deltaX);
        trackingPose.setY(trackingPose.getY() + deltaY);
        //trackingPose.setHeading(trackingPose.getHeading() + deltaAngle);  //Also applies angle bound
        double gyroHeadingReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        trackingPose.setHeadingFromGyro(gyroHeadingReading);
        //TODO:  Consider refinement to heading or position based on other sensor input
    }

    private static double calculateTranslateComponent(int translateClicks, FieldDirection fd, RobotOrientation ro, TrackingPose trackingPose){
        /**
         * Calculates the field translation component of the travel
         */
        boolean debugOn = false;
        String logTag = "BTI_calcTransComp";

        double translationComponent;
        double totalDistance = translateClicks / clicksPerInch;

        //Consider the placement of the encoder on the robot
        //This assumes that when robot front oriented with positive X direction
        //  that clicks increase with x+ and y+ field oriented vectors
        double encoderAngle = trackingPose.getHeading();

        if (debugOn) Log.d(logTag, "Entering calculateTranslateComponent for robot orientation: " + ro.name()
                + " field direction: " + fd.name());

        //Lateral encoder must shift angle to calculate effect on X and Y coordinates

        if(ro == RobotOrientation.LATERAL) {
            encoderAngle += 90;
        }



        double angleComponent;
        double encoderAngleRad = Math.toRadians(encoderAngle);

        if (debugOn) Log.d(logTag, "Encoder Angle: " + format("%.2f", encoderAngle)
                + ", travel direction: " + format("%.2f", trackingPose.getTravelDirection()));

        //If X component, need to take cos for FORWARD, sin for LATERAL
        //If Y component, need to take sin for FORWARD, cos for LATERAL
        if (fd == FieldDirection.X){
            //angleComponent = (ro==RobotOrientation.FORWARD) ? Math.cos(encoderAngleRad) : Math.sin(encoderAngleRad);
            angleComponent = Math.cos(encoderAngleRad);
        } else if (fd == FieldDirection.Y){
            //angleComponent = (ro==RobotOrientation.FORWARD) ? Math.sin(encoderAngleRad) : Math.cos(encoderAngleRad);
            angleComponent = Math.sin(encoderAngleRad);
        } else {
            angleComponent = 0;
        }
        translationComponent = totalDistance * angleComponent;

        if (debugOn) {
            Log.d(logTag, "Total Travel in " + ro.name() + " direction: " + format("%.3f", totalDistance));
            Log.d(logTag, "Exiting calculateTranslateComponent, returning " + format("%.3f", translationComponent)
                    + " for " + fd.name() + " from " + translateClicks + " translationClicks"
                    + " in " + ro.name() + " encoder direction");
            //Log.d(logTag, "Pose: " + trackingPose.toString());
        }
        return translationComponent;
    }

    private static double calculateSpinAngle(double spinClicks, ArrayList<EncoderTracker> doubleEncoders){
        boolean debugOn = true;
        String logTag = "BTI_calculateAngleCh";
        if (debugOn) Log.d(logTag, "Calculating Angle Change");
        double spinCircumference = 2 * Math.PI * doubleEncoders.get(0).spinRadius;
        double spinDistance = spinClicks / clicksPerInch;
        double spinAngle = spinDistance / spinCircumference * 360.0;

        if (debugOn) Log.d(logTag, "Spin Angle: " + format("%.2f", spinAngle) +
                " from spinClicks: " + spinClicks + " or dist: " + format("%.2f", spinDistance) +
                " over radius " + format("%.2f", doubleEncoders.get(0).spinRadius));
        return spinAngle;
    }

    private static int calculateSingleEncoderSpinClicks(double deltaAngle, double spinRadius){
        //  Calculate distance using s=r(theta)
        double spinDistance = (Math.toRadians(deltaAngle) * spinRadius);
        int spinClicks = (int) Math.round(spinDistance * clicksPerInch);
        return spinClicks;
    }

    private static int calculateSpinClicks(ArrayList<EncoderTracker> doubleEncoders) {
        /**
         * Take the difference of the 2 encoders and divide by 2
         */
        boolean debugOn = true;
        String logTag = "BTI_calculateSpinClicks";

        EncoderTracker encoderIncreases;
        EncoderTracker encoderDecreases;

        if (doubleEncoders.get(0).clickDirection == ClickDirection.STANDARD){
            encoderIncreases = doubleEncoders.get(0);
            encoderDecreases = doubleEncoders.get(1);
        } else {
            encoderIncreases = doubleEncoders.get(1);
            encoderDecreases= doubleEncoders.get(0);
        }

        //TODO:  Verify if inflation factor should be applied since the wheel orientation
        // isn't tangent to spin circumference
        int spinClicks = encoderIncreases.getIncrementalClicks() - encoderDecreases.getIncrementalClicks();

        spinClicks = (int) Math.round(spinClicks/2.0);

        return spinClicks;
    }

    private static int calculateTranslateClicks(ArrayList<EncoderTracker> doubleEncoders) {
        /**
         * Average the two encoders
         */
        boolean debugOn = false;
        String logTag = "BTI_calcTransClicks";
        if (debugOn) Log.d(logTag, "Calculating translate clicks");
        int translateClicks = 0;
        for (EncoderTracker e: doubleEncoders){
            translateClicks += e.getIncrementalClicks();
            if(debugOn) Log.d(logTag, e.getIncrementalClicks() + " incremental clicks for encoder " + e.robotOrientation + " " + e.clickDirection);
        }
        if (debugOn) Log.d(logTag, "Total Clicks: " + translateClicks);
        translateClicks = (int) Math.round(translateClicks/2.0);
        if (debugOn) Log.d(logTag, "Averaged Value: " + translateClicks);
        return translateClicks;
    }

    private static ArrayList<EncoderTracker> getDoubleEncoders(RobotOrientation robotOrientation){
        /**
         * Returns 2 properly setup encoders or an empty list if problem detected
         */
        boolean debugOn = false;
        String logTag = "BTI_updatePose...Three";
        RobotOrientation doubleOrientation = robotOrientation;

        ArrayList<EncoderTracker> doubleEncoders = new ArrayList<>();
        if (doubleEncoders.size()>0) doubleEncoders.clear();

        //Make sure that there are 3 encoders
        if (getEncoderTrackerCount() != 3) {
            Log.d(logTag, "Not 3 encoders");
        } else {
            for (EncoderTracker e: encoders){
                if(e.robotOrientation == doubleOrientation) doubleEncoders.add(e);
            }
        }

        if (doubleEncoders.size() > 0 && doubleEncoders.size() != 2){
            if(debugOn) Log.d(logTag, "Incorrect number of double encoders found");
            doubleEncoders.clear();
        }

        if (!verifyDoubleEncoderSpinBehavior(doubleEncoders)){
            if (debugOn) Log.d(logTag, "Prematurely exiting updatePoseUsingThreeEncoders");
            //  Encoder direction not spin behavior not correct, clear the list
            doubleEncoders.clear();
        }

        return doubleEncoders;

    }

    private static EncoderTracker getSingleEncoder(RobotOrientation ro){
        EncoderTracker singleEncoder = null;
        for (EncoderTracker e: encoders){
            if(e.robotOrientation != ro){
                singleEncoder = e;
                break;
            }
        }
        return singleEncoder;
    }

    private static boolean verifyDoubleEncoderSpinBehavior(ArrayList<EncoderTracker> doubleEncoders){
        boolean debugOn = false;
        String logTag = "BTI_verifyDouble...Spin";
        boolean result;

        boolean increasesCovered = false;
        boolean decreasesCovered= false;

        for (EncoderTracker e: doubleEncoders){
            if (e.spinBehavior == SpinBehavior.INCREASES_WITH_ANGLE){
                increasesCovered = !increasesCovered;
            } else if (e.spinBehavior == SpinBehavior.DECREASES_WITH_ANGLE){
                decreasesCovered = !decreasesCovered;
            }
        }

        if (!increasesCovered | !decreasesCovered) {
            Log.d(logTag, "Encoders not setup for opposite reaction to spin");
            result = false;
        } else result = true;

        return result;
    }

    public static void updateVirtualEncoders(Double outputSignal,Long loopDuration, TrackingPose currentPose){
        Double distance = VirtualEncoder.calculateSimulatedDistance(outputSignal, loopDuration);

        Double angleHeading = currentPose.getHeading();
        Double angleTravel = currentPose.getTravelDirection();
        Double encoderAngle = angleTravel - angleHeading;

        for (EncoderTracker e: encoders){
            if (e.robotOrientation == RobotOrientation.LATERAL){
                encoderAngle += 90;
            }
            Double encoderAngleRad = Math.toRadians(encoderAngle);
            e.virtualEncoder.simulateLoopOutput(distance, encoderAngleRad, RobotOrientation.FORWARD);
        }
    }

    public static void updateVirtualEncoders(double translateSignal, double travelDirection, double spinSignal, Long loopDuration, TrackingPose currentPose){
        double distance = VirtualEncoder.calculateSimulatedDistance(translateSignal, loopDuration);
        double spinDistance = VirtualEncoder.calculateSimulatedRotation(spinSignal, loopDuration);

        double angleHeading = currentPose.getHeading();
        double encoderAngle = travelDirection - angleHeading;

        for (EncoderTracker e: encoders){
            //  Apply the orientation to the encoder
            /*
            if (e.robotOrientation == RobotOrientation.LATERAL){
                encoderAngle += 90;
            }

             */
            double encoderAngleRad = Math.toRadians(encoderAngle);

            //Assign the correct sign to the spin distance
            int spinSign = (e.spinBehavior == SpinBehavior.INCREASES_WITH_ANGLE) ? 1 : -1;

            //simulate the loop event to the virtual encoder
            e.virtualEncoder.simulateLoopOutput(distance, encoderAngleRad, e.robotOrientation, (spinDistance * spinSign));
        }
    }


    public static void purgeExistingEncoderTrackers(){
        if (encoders.size() > 0) encoders.clear();
    }

    public static void updateEncoderCurrentClicks(){
        boolean debugOn = true;
        String logTag = "BTI_updateEndCurClicks";

        for(EncoderTracker e: encoders){
            int oldValue = e.currentClicks;
            e.currentClicks = e.getClicks();
            if (debugOn) Log.d(logTag, e.robotOrientation.name() + " encoder was " + oldValue
                            + " and is now " + e.currentClicks);
        }
    }

    /***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
    //****************************************************************/


    public static Integer getEncoderTrackerCount(){
        return encoders.size();
    }

    public void reverseClickDirection(){
        this.clickDirection = ClickDirection.REVERSE;
    }

    public void setSpinBehavior(SpinBehavior spinBehaviorIn){
        this.spinBehavior = spinBehaviorIn;
    }

    public double getSpinRadius(){return this.spinRadius;}
    public void setSpinRadius(double radius){
        this.spinRadius = radius;
    }

    public void setWheelDiameter(double diam){
        this.wheelDiameter = diam;
    }

    public void setClickDirection(ClickDirection clickDirection){
        this.clickDirection = clickDirection;
    }

    /***************************************************************
     //******    CLASS METHODS
     //****************************************************************/

    public Integer getClicks(){
        Integer clicks;
        //  Depending on whether the encoder is virtual (motor is null) or real
        //  It runs a different command

        if (this.motor == null){
            clicks = this.virtualEncoder.getClicks();
        } else {
            clicks = this.motor.getCurrentPosition();
        }
        return clicks;
    }

    private int getIncrementalClicks(){
        int incrementalClicks = this.getClicks() - this.currentClicks;
        //int sign = (this.clickDirection == ClickDirection.REVERSE) ? -1 : 1;
        return (incrementalClicks);
    }


    @Override
    public String toString(){
        String outputString;
        if (this.motor != null) {
            outputString =  "Real encoder " + this.robotOrientation.name()
                    + " Motor Port: " + this.motor.getPortNumber() + " Clicks: "
                    + this.getClicks() + "= Distance: " + format("%.2f", Math.abs(this.getClicks() / clicksPerInch))
                    + ", clickDirection: " + this.clickDirection.name()
                    + ", spinBehavior: " + this.spinBehavior.name();
        } else {
            outputString = "Virtual Encoder, Orientation: " + this.robotOrientation.name()
                    + this.clickDirection.name() + " currentClicks: " + currentClicks
                    + " New Reading: " + this.getClicks() + " Incremental: " + this.getIncrementalClicks();
        }
        return outputString;
    }

}


