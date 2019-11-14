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
        this.yCenter = 72.0 - 18.0 - this.width/2.0;
        //starts 4" from top wall, x coord doesn't change
        this.xCenter = 72.0 - 4.0 - this.length / 2.0;
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

    public Pose getSkyStoneDumpingPose(Double offsetDistance){
        return new Pose(xCenter-height/2.0 - offsetDistance, yCenter - width/2.0, orientationDegrees);
    }
}
