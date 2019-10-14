package org.firstinspires.ftc.teamcode;

public class Pose {
    /*
    *****   Pose represents the robots position AND heading relative to field coordinates
    *       Units:
    *       x       -> Inches, 0 at center of field, positive running towards foundation side
    *       y       -> Inches, 0 at center of field, positive running towards blue side
    *       heading -> Degrees, 0 inline with x-axis, positive CCW when viewed from top
     */


    private Double x;
    private Double y;
    private Double heading;
    private static final double overallFieldLength = 72.0;

    public enum StartingPose{
        RED_FOUNDATION (12.0, -27.0, 90.0)
        , BLUE_FOUNDATION (12.0, 27.0, -90.0)
        , RED_QUARRY (-12.0, -27.0, 90.0)
        , BLUE_QUARRY (-12.0, 27.0, -90.0);



        private double xStart;
        private double yStart;
        private double headingStart;

        private StartingPose(double xInput, double yInput, double headingInput){
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

    public Pose (Double xInput, Double yInput, Double headingInput){
        this.x = xInput;
        this.y = yInput;
        this.heading = headingInput;
    }

    public Pose(StartingPose startingPose) {
        this.x = startingPose.getxStart();
        this.y = startingPose.getyStart();
        this.heading = startingPose.getHeadingStart();
    }


    //Create getters and setters
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

    public void setHeading(Double heading) {
        this.heading = heading;
    }

    public Double getXError(Pose targetPose){
        Double xError = targetPose.getX() - this.x;
        return xError;
    }

    public Double getYError(Pose targetPose){
        Double yError = targetPose.getY() - this.y;
        return yError;
    }


}
