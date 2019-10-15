package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class EncoderTracker {
    //  EncoderTracker records and computes the differences in encoder positions
    //  and translates them back into x,y (and maybe heading eventually)

    //***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88

    private DcMotor motor;                      //motor that the encoder is attached to
    private RobotOrientation robotOrientation;  //orientation of the encoder relative to robot reference frame
                                                //  FORWARD
                                                //  LATERAL -> Left is positive

    private Integer currentClicks;              //  Number of clicks for current iteration

    //***************************************************************88
    //******    STATIC VARIABLES
    //***************************************************************88
    private static Double clicksPerInch = 434.82;   //  Clicks per inch of travel

    //A list of all the encoders being tracked
    private static ArrayList<EncoderTracker> encoders = new ArrayList<>();

    //***************************************************************88
    //******    ENUMERATIONS
    //***************************************************************88

    public enum RobotOrientation{
        FORWARD,
        LATERAL
    }

    //***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
    //***************************************************************88


    //***************************************************************88
    //******    CONSTRUCTORS
    //***************************************************************88

    public EncoderTracker (DcMotor motor, RobotOrientation robotOrientation){
        this.motor = motor;
        this.robotOrientation = robotOrientation;
        this.currentClicks = 0;
        encoders.add(this);
    }


    //***************************************************************88
    //******    METHODS
    //***************************************************************88

    public static void getNewPose (TrackingPose trackingPose){
        Integer newClicks;                   //new encoder position for this iteration
        Double newX = trackingPose.getX();   //starting position for x
        Double newY = trackingPose.getY();   //starting position for y
        Double encoderAngle;                 //to track field-oriented encoder orientation
        Double totalDistance;                //linear distance traveled before x and y components calculated

        // TODO: 10/13/2019 Add consideration for rotation

        //  Loop through the encoders to figure out how much
        for(EncoderTracker e: encoders){
            //Calculate distance traveled for encoder
            newClicks = e.motor.getCurrentPosition();
            totalDistance = (newClicks - e.currentClicks) * clicksPerInch;

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

            //  Set the new positions by adding deltas to corresponding coordinate
            //  These are cumulative for each encoder analyzed
            newX += deltaX;
            newY += deltaY;

            //  Update the encoder position for next iteration of the loop
            e.currentClicks = newClicks;
        }

        trackingPose.setX(newX);
        trackingPose.setY(newY);
        trackingPose.setHeading(newY);
    }

}
