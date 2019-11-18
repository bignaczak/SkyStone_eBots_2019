package org.firstinspires.ftc.teamcode;

public class Foundation {
    /****************************************************************
     //******    CLASS VARIABLES
     //***************************************************************/

    private double length;
    private double width;
    private double height;
    private double xCenter;
    private double yCenter;
    private double orientationDegrees;  //this is the line connecting midpoints of width walls

    /*****************************************************************
     //******    CONSTRUCTORS
     //****************************************************************/

    public Foundation(){
        this.length = 34.5;
        this.width = 18.5;
        this.height = 2.25;
        //Assume 18" robot pulls it toward wall
        this.yCenter = 41.75;
        //starts 4" from top wall, x coord doesn't change
        this.xCenter = 48.0;
        this.orientationDegrees = 0;
    }
    public Foundation(eBotsAuton2019.Alliance alliance){
        this();
        if(alliance == eBotsAuton2019.Alliance.RED){
            //Flip the y coordinate if red
            this.yCenter *= -1;
        }
    }
    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/

    public Pose getSkyStoneDumpingPose(double offsetDistance){
        return new Pose(xCenter-length/2.0 - offsetDistance, yCenter, orientationDegrees);
    }
    public Pose getSkyStoneDumpingPose(){
        double offsetDistance = 19.0;
        return new Pose(xCenter-length/2.0 - offsetDistance, yCenter, orientationDegrees);
    }

    public Pose getPoseAfterPlaceSkystone(TrackingPose endPose, eBotsAuton2019.Alliance alliance){
        double incrementSign = (alliance == eBotsAuton2019.Alliance.BLUE) ? -1.0 : 1.0;
        return new Pose (endPose.getX() - 4.0, endPose.getY() + (10.0 * incrementSign), 0.0);
    }


}
