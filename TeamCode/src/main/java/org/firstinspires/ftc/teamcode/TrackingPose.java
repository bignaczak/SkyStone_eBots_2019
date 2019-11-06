package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

public class TrackingPose extends Pose {
    /**  This is used for a dynamic pose that is used to track the robots position
        This is unique because it also has an error object attached to it
        That tracks progress towards the target pose
        Each instance of a trackingPose represents one leg of an auton path
     */

    //***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88
    private Pose targetPose;        // This is the targeted destination pose
    private PoseError poseError;    // Error between this pose and a target pose

    private Double initialGyroOffset;  //When the robot starts, the gyro is zero in whichever orientation the robot is facing
                                        //  So if robot always faces the center, from the red side, the gyro will read 0 when the
                                        //  robot is actually facing +90°
                                        //  This captures the rotation required to bring the field coordinates frame in line with the
                                        //  the robot coordinate system

    private Boolean headingErrorLocked; //  In order to allow for overshoot, the headingError
                                        //  direction needs to be locked when close to our target
                                        //  This happens when the signal is not saturated


    private Double travelDirection;     //  This is the direction that the robot is traveling
                                        //  This is usually the same as headingError
                                        //  But becomes locked when near target to allow for overshoot

    //***************************************************************88
    //******    CONSTRUCTORS
    //***************************************************************88

    public TrackingPose(){}

    public TrackingPose(Pose startingPose, Pose targetPose) {
        this.setX(startingPose.getX());
        this.setY(startingPose.getY());
        this.setHeading(startingPose.getHeading());
        this.targetPose = targetPose;
        this.headingErrorLocked = false;
        this.poseError = new PoseError(this);
        this.travelDirection = this.getHeadingError();
    }

    //***************************************************************88
    //******    GETTERS AND SETTERS
    //***************************************************************88
    public Double getTravelDirection() {return travelDirection;}
    public Double getTravelDirectionRad() {return Math.toRadians(travelDirection);}
    public Boolean isHeadingErrorLocked(){return headingErrorLocked; }
    public Pose getTargetPose(){return this.targetPose;}
    public Double getInitialGyroOffset(){return this.initialGyroOffset;}


    //***************************************************************88
    //******    INSTANCE METHODS
    //***************************************************************88

    public void setInitialGyroOffset(Double gyroReading){
        //  This is run right after creating the tracking pose
        //  This captures the rotation required to bring the field coordinates frame in line with the
        //  the robot coordinate system
        initialGyroOffset = this.getHeading() - gyroReading;
    }

    public void copyInitialGyroOffsetBetweenLegs(Double previousInitialGyroOffset){
        /**  This is intended to copy the initialGyro offset from the TrackingPose used
         *   in the first leg to each successive TrackingPose of the auton path
         */
        this.initialGyroOffset = previousInitialGyroOffset;
    }



    public void calculatePoseError(){
        //  Calculate the pose error between the new position and target Pose
        this.poseError.calculateError(this);
        //poseError = new PoseError(this, targetPose);
    }

    //  Robot local heading determines which way the robot needs to move
    public Double getGyroDriveDirection(){
        Double gyroDriveDirection = this.getHeadingError() - this.initialGyroOffset;
        gyroDriveDirection = applyAngleBound(gyroDriveDirection);
        return gyroDriveDirection;
    }



    public Double getGyroDriveDirectionRad(){
        return Math.toRadians(this.getGyroDriveDirection());
    }

    public Double getGyroCurrentReading(){
        return this.getHeadingError() - initialGyroOffset;
    }

    public void setHeadingFromGyro(Double gyroHeading){
        Double fieldHeading = gyroHeading + this.initialGyroOffset;
        fieldHeading = applyAngleBound(fieldHeading);
        this.setHeading(fieldHeading);
    }

    public void setHeadingFromGyro(Double gyroHeading, Double previousInitialGyroOffset){
        Double fieldHeading = gyroHeading + previousInitialGyroOffset;
        fieldHeading = applyAngleBound(fieldHeading);
        this.setHeading(fieldHeading);
    }


    public Double getSignedError(){
        return this.poseError.getSignedError();
    }

    public Double getErrorMagnitude(){
        return this.poseError.getMagnitude();
    }

    public Double getHeadingError(){
        return this.poseError.getHeading();
    }

    public Double getHeadingErrorRad(){
        return Math.toRadians(this.getHeadingError());
    }

    public Double getErrorControlValue(){
        //This passes the request down to the poseError object
        return this.poseError.getErrorControlValue();
    }

    public Double getXError(){
        return this.targetPose.getX() - this.getX();
    }

    public Double getYError(){
        return this.targetPose.getY() - this.getY();
    }

    public Double getErrorSum(){
        return this.poseError.getErrorSum();
    }

    public void updateErrorSum(Boolean isSaturated){
        this.poseError.calculateErrorSum(isSaturated);
    }

    public void setHeadingErrorLocked(Boolean isSaturated, Boolean driveSignalSignChange){
        if (!isSaturated && driveSignalSignChange){
            headingErrorLocked = false;
        } else if (!isSaturated){
            headingErrorLocked = true;
        } else {
            headingErrorLocked = false;
        }
    }

    public void updateTravelDirection(){
        if (!headingErrorLocked) travelDirection = getHeadingError();
    }



    public String printError(){
        return "Error: " + format("%.2f", this.getSignedError()) +
                " @ " + format("%.2f", this.getHeadingError()) +
                ", errorSum: " + format("%.2f", this.getErrorSum());
    }

    @Override
    public String toString(){
        String lockCondition;
        if (isHeadingErrorLocked()){
            lockCondition = "[LOCKED]";
        } else {
            lockCondition = "[UNLOCKED]";
        }
        return super.toString() + "-->" + format("%.1f", this.travelDirection) +
               " " + lockCondition;
    }

    //***************************************************************88
    //******    STATIC METHODS
    //***************************************************************88

    public static Double applyAngleBound (Double inputAngle){
        while (inputAngle > 180){
            inputAngle -= 360;
        }
        while (inputAngle <= -180){
            inputAngle += 360;
        }
        return inputAngle;
    }

}
