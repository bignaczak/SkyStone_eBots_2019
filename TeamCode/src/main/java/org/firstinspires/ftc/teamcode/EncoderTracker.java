package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

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

    /***************************************************************88
    //******    STATIC VARIABLES
    //****************************************************************/
    private static Double clicksPerInch = 434.82;   //  Clicks per inch of travel

    //A list of all the encoders being tracked
    private static ArrayList<EncoderTracker> encoders = new ArrayList<>();

    /***************************************************************88
    //******    ENUMERATIONS
    //****************************************************************/

    public enum RobotOrientation{
        FORWARD,
        LATERAL
    }

    /***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
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


    public static Integer getEncoderTrackerCount(){
        return encoders.size();
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
        encoders.add(this);
    }

    public EncoderTracker (VirtualEncoder virtualEncoder, RobotOrientation robotOrientation){
        this.motor = null;
        this.virtualEncoder = virtualEncoder;
        this.robotOrientation = robotOrientation;
        this.currentClicks = this.getClicks();
        this.cumulativeDistance = 0.0;
        this.cumulativeClicks =0;
        encoders.add(this);
    }

    /***************************************************************
    //******    STATIC METHODS
    //****************************************************************/

    public static void getNewPose (TrackingPose trackingPose){
        boolean debugOn = true;
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

        //  Loop through the encoders to figure out how much
        for (EncoderTracker e: encoders){
            //Calculate distance traveled for encoder
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
            e.virtualEncoder.simulateLoopOutput(distance, encoderAngleRad);
        }
    }

    public static void purgeExistingEncoderTrackers(){
        if (encoders.size() > 0) encoders.clear();
    }

    public static void updateEncoderCurrentClicks(){
        for(EncoderTracker e: encoders){
            e.currentClicks = e.getClicks();
        }
    }
/***************************************************************
 //******    CLASS METHODS
 //****************************************************************/

    @Override
    public String toString(){
        String outputString;
        if (this.motor != null) {
            outputString =  "Real encoder " + this.robotOrientation.name()
                    + " Motor Port: " + this.motor.getPortNumber() + " Clicks: "
                    + this.motor.getCurrentPosition();
        } else {
            outputString = "Virtual Encoder, Orientation: " + this.robotOrientation.name()
                    + " Clicks: " + this.getClicks();
        }
        return outputString;
    }

}


