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
    private Orientation orientation;  //this is the line connecting midpoints of width walls

    private final double FIELD_WIDTH = 69.5;

    /*****************************************************************
     //******    CONSTRUCTORS
     //****************************************************************/

    public Foundation(){
        this.length = 34.5;
        this.width = 18.5;
        this.height = 2.25;
        //Assume 18" robot pulls it toward wall
        this.yCenter = FIELD_WIDTH - 10.0 - (this.width/2.0);  //42.25;
        //starts 4" from top wall, x coord doesn't change
        this.xCenter = FIELD_WIDTH - (this.length/2.0 +4.0);  //48.25 inches
        this.orientation = Orientation.DEFAULT;
    }
    public Foundation(eBotsAuton2019.Alliance alliance){
        this();
        if(alliance == eBotsAuton2019.Alliance.RED){
            //Flip the y coordinate if red
            this.yCenter *= -1;
        }
    }

    //***********************************************************
    //******        ENUMS
    //***********************************************************
    public enum Orientation {
        DEFAULT ,
        ROTATED
    }


    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/

    //***********************************************************
    //******        CLASS METHODS
    //***********************************************************

    public void rotateFoundation(Orientation orientation, eBotsAuton2019.Alliance alliance){
        if (orientation == Orientation.ROTATED){
            this.xCenter = FIELD_WIDTH - width/2.0;
            this.yCenter = FIELD_WIDTH - length/2.0 - 12.0;  //Assume 12 inches from wall
            this.orientation = Orientation.ROTATED;
        } else {
            //Assume 18" robot pulls it toward wall
            this.yCenter = FIELD_WIDTH - 18.0 - (this.width/2.0);  //42.25;
            //starts 4" from top wall, x coord doesn't change
            this.xCenter = FIELD_WIDTH - (this.length/2.0 +4.0);  //48.25 inches
            this.orientation = Orientation.DEFAULT;
        }
        if(alliance == eBotsAuton2019.Alliance.RED){
            //Flip the y coordinate if red
            this.yCenter *= -1;
        }


    }
    public Pose getSkyStoneDumpingPose(double offsetDistance){
        return new Pose(xCenter-length/2.0 - offsetDistance, yCenter, 0.0);
    }
    public Pose getSkyStoneDumpingPose(){
        double offsetDistance = 19.0;
        return new Pose(xCenter-length/2.0 - offsetDistance, yCenter, 0.0);
    }

    public Pose getSkyStoneDumpingPose(eBotsAuton2019.Alliance alliance){
        double offsetDistance = 25.0;
        double xCoord;
        double yCoord;
        if (orientation == Orientation.DEFAULT){
            xCoord  = this.xCenter-this.length/2.0 - offsetDistance;
            yCoord = this.yCenter;
        } else{
            xCoord = this.xCenter - this.width/2.0 - offsetDistance;
            double sign = (alliance == eBotsAuton2019.Alliance.RED) ? 1.0 : -1.0;
            yCoord = this.yCenter + (this.length/3.0 * sign);
        }
        return new Pose(xCoord, yCoord, 0.0);
    }



    public Pose getSkyStoneParkingPose(eBotsAuton2019.Alliance alliance){
        double offsetDistance = 19.0;
        double tapeOrientation = -15.0;  // for blue
        if (alliance == eBotsAuton2019.Alliance.RED) tapeOrientation = -tapeOrientation;
        return new Pose(xCenter-length/2.0 - offsetDistance, yCenter, tapeOrientation);
    }


    public Pose getPoseAfterPlaceSkystone(TrackingPose endPose, eBotsAuton2019.Alliance alliance){
        double incrementSign = (alliance == eBotsAuton2019.Alliance.BLUE) ? -1.0 : 1.0;
        return new Pose (endPose.getX() - 4.0, endPose.getY() + (10.0 * incrementSign), 0.0);
    }

}
