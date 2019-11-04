package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

public class Pose {
    //****   Pose represents the robots position AND heading relative to field coordinates

    //***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88

    private Double x;               // in Inches, 0 at center of field, positive running towards foundation side
    private Double y;               // In Inches, 0 at center of field, positive running towards blue side
    private Double heading;         // Degrees, 0 inline with x-axis, positive CCW when viewed from top
    //private PoseError poseError;    // Error between this pose and a target pose
    private PostMoveActivity postMoveActivity;      //  To actuate actions in Auton

    //***************************************************************88
    //******    STATIC VARIABLES
    //***************************************************************88
    private static final double overallFieldLength = 72.0;  //in inches


    //***************************************************************88
    //******    ENUMERATIONS
    //***************************************************************88
    public enum StartingPose{
        RED_FOUNDATION (-12.0, -62.0, 90.0)
        , BLUE_FOUNDATION (22.0, 62.0, -90.0)
        , RED_QUARRY (-12.0, -62.0, 90.0)
        , BLUE_QUARRY (-12.0, 62.0, -90.0);

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
        CYCLE_CLAW
    }

    //***************************************************************88
    //******    CONSTRUCTORS
    //***************************************************************88

    public Pose(){}     //Default constructor

    public Pose (Double xInput, Double yInput, Double headingInput){
        this.x = xInput;
        this.y = yInput;
        this.heading = headingInput;
        //this.poseError = new PoseError();
        this.postMoveActivity = PostMoveActivity.NONE;
    }

    public Pose (Double xInput, Double yInput, Double headingInput, PostMoveActivity postMoveActivity){
        this.x = xInput;
        this.y = yInput;
        this.heading = headingInput;
        this.postMoveActivity = postMoveActivity;
        //this.poseError = new PoseError();
    }


    //  When using a pre-defined StartingPose from the enumeration
    public Pose(StartingPose startingPose) {
        this.x = startingPose.getxStart();
        this.y = startingPose.getyStart();
        this.heading = startingPose.getHeadingStart();
        this.postMoveActivity = PostMoveActivity.NONE;

    }




    //***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
    //***************************************************************88
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

    //***************************************************************88
    //******    METHODS
    //***************************************************************88

    @Override
    public String toString(){
        return "(" + format("%.2f",x) + " ," + format("%.2f",y) + " @ " + format("%.2f",heading) + ")";
    }

}
