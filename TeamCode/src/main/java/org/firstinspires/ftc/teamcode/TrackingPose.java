package org.firstinspires.ftc.teamcode;

public class TrackingPose extends Pose {
    //  This is used for a dynamic pose that is used to track the robots position
    //  This is unique because it also has an error object attached to it
    //  That tracks progress towards the target pose

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
    }

    //***************************************************************88
    //******    INSTANCE METHODS
    //***************************************************************88

    public void setInitialGyroOffset(Double gyroReading){
        //  This is run right after creating the tracking pose
        //  This captures the rotation required to bring the field coordinates frame in line with the
        //  the robot coordinate system
        initialGyroOffset = this.getHeading() - gyroReading;
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

    public void setHeadingFromGyro(Double gyroHeading){
        Double fieldHeading = gyroHeading + this.initialGyroOffset;
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

    public Boolean isHeadingErrorLocked(){
        return headingErrorLocked;
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
