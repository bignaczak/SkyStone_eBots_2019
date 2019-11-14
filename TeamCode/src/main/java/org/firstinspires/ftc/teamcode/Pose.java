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
        CYCLE_CLAW,
        SCAN_FOR_SKYSTONE
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

    //***************************************************************88
    //******    METHODS
    //***************************************************************88

    @Override
    public String toString(){
        return "(" + format("%.2f",x) + " ," + format("%.2f",y) + " @ " + format("%.2f",heading)
                + "-->" + this.postMoveActivity.name() + ")";
    }

}
