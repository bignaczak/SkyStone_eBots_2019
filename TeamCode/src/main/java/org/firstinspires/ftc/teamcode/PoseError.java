package org.firstinspires.ftc.teamcode;

public class PoseError {
    //  PoseError represents the difference in position and heading
    //  between a given pose and a target pose

    //***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88

    private Double magnitude;   //  magnitude is in inches

    private Integer signOfMagnitude;    //  For PID control loop, it is useful to associate
                                        //    a sign to the Error angle
                                        //    NEGATIVE ->   -180 < theta <= 0   (yep, zero is negative now)
                                        //    POSITIVE ->      0 < theta <= 180

    private Double errorSum;    //  Cumulative error for the Integrator portion of PID controller

    private Double heading;     //  Which way the robot needs to travel relative to the field
                                //    to reach target position


    //***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
    //***************************************************************88
    public Double getMagnitude() {return magnitude; }
    public Integer getSignOfMagnitude() {return signOfMagnitude;}
    public Double getHeading() {return heading;}
    public Double getErrorSum() {return errorSum;}


    //***************************************************************88
    //******    CONSTRUCTORS
    //***************************************************************88
    public PoseError(){
        this.magnitude = 0.0;
        this.signOfMagnitude = -1;
        this.heading = 0.0;
        this.errorSum = 0.0;
    }

    public PoseError(TrackingPose currentPose) {
        calculateError(currentPose);

        //  Set the errorSum to zero when instantiated
        this.errorSum = 0.0;
    }
    //***************************************************************88
    //******    METHODS
    //***************************************************************88


    public Double getSignedError(){
        return (this.magnitude * this.signOfMagnitude);
    }

    public Integer getSignOfErrorSum(){
        return (errorSum <= 0) ? -1 : 1;
    }

    public void calculateError(TrackingPose currentPose){
        double xError = currentPose.getXError();
        double yError = currentPose.getYError();

        this.magnitude = Math.hypot(xError, yError);

        //Calculate heading error if it is not locked
        //  It gets locked to allow the Integrator (ErrorSum) to unwind
        //  Which allows for overshoot prior to changing directions
        if (!currentPose.isHeadingErrorLocked()) {
            //  atan2 returns an angle between -180 < theta <= 180
            this.heading = Math.toDegrees(Math.atan2(yError, xError));
        }
        //  For PID control loop, it is useful to associate a signOfMagnitude to the angle
        //    NEGATIVE ->   -180 < theta <= 0       (yep, zero is negative now)
        //    POSITIVE ->      0 < theta <= 180
        if (this.heading <= 0){
            this.signOfMagnitude = -1;
        } else {
            this.signOfMagnitude = 1;
        }

    }

    public void calculateErrorSum(Boolean isSaturated){
        //Compute a new ErrorSum for the Integrator
        //  if BOTH of the following conditions are met, don't add the integrator
        //  -->signal is saturated [isSaturated]
        //  -->error sign same as error Sum (evalutes to true if errorSum 0) [isErrorSameSignAsErrorSum]

        Boolean isErrorSameSignAsErrorSum;
        if (this.errorSum >= 0 && this.getSignedError() >= 0){
            isErrorSameSignAsErrorSum = true;
        } else if (this.errorSum<= 0 && this.getSignedError() <= 0){
            isErrorSameSignAsErrorSum = true;
        } else {
            isErrorSameSignAsErrorSum = false;
        }

        if (isSaturated && isErrorSameSignAsErrorSum){
            //  Don't add the error sum
        } else {
            this.errorSum += this.getSignedError();
        }
    }
}
