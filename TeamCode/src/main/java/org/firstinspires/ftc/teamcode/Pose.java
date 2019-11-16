package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

/**
 * Pose represents the robots position AND heading relative to field coordinates
 * It can also carry instructions on what to do after an auton travel leg
 */
public class Pose {

    /****************************************************************
    //******    CLASS VARIABLES
    //***************************************************************/

    private Double x;               // in Inches, 0 at center of field, positive running towards foundation side
    private Double y;               // In Inches, 0 at center of field, positive running towards blue side
    private Double heading;         // Degrees, 0 inline with x-axis, positive CCW when viewed from top
    private PostMoveActivity postMoveActivity;      //  To actuate actions in Auton

    /***************************************************************
    //******    STATIC VARIABLES
    //****************************************************************/
    private static final double overallFieldLength = 72.0;  //in inches


    /***************************************************************
    //******    ENUMERATIONS
    //***************************************************************/
    public enum StartingPose{
        RED_FOUNDATION (-12.0, -62.0, 90.0)
        , BLUE_FOUNDATION (39.0, 62.0, 90.0)
        , RED_QUARRY (-12.0, -62.0, 90.0)
        , BLUE_QUARRY (-12.0, 62.0, -90.0)
        , FOUNDATION(39.0, 62.0, 90.0)
        , QUARRY (-39.0, 62.0, -90.0);

        private double xStart;
        private double yStart;
        private double headingStart;

        StartingPose(double xInput, double yInput, double headingInput){
            this.xStart = xInput;
            this.yStart = yInput;
            this.headingStart = headingInput;
        }

        public double getxStart() {
            return xStart;
        }
        public double getyStart() {
            return yStart;
        }
        public double getHeadingStart() {
            return headingStart;
        }
    }

    //***************************************************************88

    public enum PostMoveActivity{
        NONE,
        LOWER_RAKE,
        RAISE_RAKE,
        RAISE_LIFTER,
        CYCLE_CLAW,
        SCAN_FOR_SKYSTONE,
        RAISE_LIFTER_TO_EXTEND_ARM,
        EXTEND_ARM_THEN_LOWER_LIFTER
    }


    /*****************************************************************
    //******    CONSTRUCTORS
    //****************************************************************/

    public Pose(){}     //Default constructor

    public Pose (Double xInput, Double yInput, Double headingInput){
        this.x = xInput;
        this.y = yInput;
        this.heading = headingInput;
        this.postMoveActivity = PostMoveActivity.NONE;
    }

    public Pose (Double xInput, Double yInput, Double headingInput, PostMoveActivity postMoveActivity){
        this(xInput, yInput, headingInput);

        //Update the post move activity
        this.postMoveActivity = postMoveActivity;
    }

    //  When using a pre-defined StartingPose from the enumeration
    public Pose(StartingPose startingPose) {
        this(startingPose.getxStart(), startingPose.getyStart(), startingPose.getHeadingStart());
    }



    /*****************************************************************
    //******    SIMPLE GETTERS AND SETTERS
    //****************************************************************/
    public Double getX() {
        return x;
    }
    public void setX(Double x) {
        this.x = x;
    }
    public Double getY() {
        return y;
    }
    public void setY(Double y) {
        this.y = y;
    }
    public Double getHeading() {
        return heading;
    }
    public Double getHeadingRad(){
        return Math.toRadians(heading);
    }
    public PostMoveActivity getPostMoveActivity(){
        return this.postMoveActivity;
    }


    public void setHeading(Double heading) {
        /** Sets heading, but makes sure it is within the legal bounds
         *  which is -180 < heading <= 180
         */
        heading = TrackingPose.applyAngleBound(heading);
        this.heading = heading;
    }

    public void setPostMoveActivity(PostMoveActivity activity){
        this.postMoveActivity = activity;
    }

    /***************************************************************88
    //******    Class METHODS
    //***************************************************************/

    @Override
    public String toString(){
        return "(" + format("%.2f",x) + " ," + format("%.2f",y) + " @ " + format("%.2f",heading)
                + "-->" + this.postMoveActivity.name() + ")";
    }

    /***************************************************************88
     //******    Static METHODS
     //***************************************************************/
    public static Pose getBridgeParkPose(TrackingPose trackingPose, eBotsAuton2019.Alliance alliance){
        double parkX = 4.0;     //Center of the robot 4" from midline in building zone
        double parkY = 39.0;    //Near the inner edge of the field
        double heading = trackingPose.getHeading();

        if(trackingPose.getX() < 0){
            parkX = - parkX;
        }
        if (alliance == eBotsAuton2019.Alliance.RED){
            parkY = -parkY;
        }
        return new Pose(parkX, parkY, heading);
    }

}
