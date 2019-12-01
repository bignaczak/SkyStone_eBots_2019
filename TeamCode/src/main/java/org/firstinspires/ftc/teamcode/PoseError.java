package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

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
    private Double xErrorSum;    //  and for x component
    private Double yErrorSum;    //  and for y component
    private Double spinErrorSum;    //  and for spin




    private Double heading;     //  Which way the robot needs to travel relative to the field
                                //    to reach target position


    private Double errorControlValue;  //This is used to determine the required accuracy to exit a control loop
                                    //  It combines the present error with the ErrorSum (including sign)


    //***************************************************************88
    //******    SIMPLE GETTERS AND SETTERS
    //***************************************************************88
    public Double getMagnitude() {return magnitude; }
    public Integer getSignOfMagnitude() {return signOfMagnitude;}
    //TODO:  Change this heading to travelDirection, or something else less confusing with trackingPose heading
    public Double getHeading() {return heading;}
    public Double getXErrorSum() {return xErrorSum;}
    public Double getYErrorSum() {return yErrorSum;}
    public Double getSpinErrorSum() {return spinErrorSum;}
    public Double getErrorSum() {return errorSum;}
    public Double getErrorControlValue() {return errorControlValue;}
    public void setErrorSum(double newValue){this.errorSum = newValue;}


    //***************************************************************88
    //******    CONSTRUCTORS
    //***************************************************************88
    public PoseError(){
        this.magnitude = 0.0;
        this.signOfMagnitude = -1;
        this.heading = 0.0;
        resetErrorSums();
        this.calculateErrorControlValue();
    }



    public PoseError(TrackingPose currentPose) {
        //  Set the errorSum to zero when instantiated
        resetErrorSums();
        calculateError(currentPose);


    }
    //***************************************************************88
    //******    METHODS
    //***************************************************************88

    public void resetErrorSums(){
        this.errorSum = 0.0;
        this.xErrorSum = 0.0;
        this.yErrorSum = 0.0;
        this.spinErrorSum = 0.0;
    }
    public Double getSignedError(){
        return (this.magnitude * this.signOfMagnitude);
    }

    public Integer getSignOfErrorSum(){
        return (errorSum <= 0) ? -1 : 1;
    }

    public void calculateError(TrackingPose currentPose){
        double xError = currentPose.getXError();
        double yError = currentPose.getYError();

        this.magnitude = Math.hypot(Math.abs(xError), Math.abs(yError));

        //Calculate heading error if it is not locked
        //  It gets locked to allow the Integrator (ErrorSum) to unwind
        //  Which allows for overshoot prior to changing directions
        //  This is now handledd by travelDirection in TrackingPose class
        //if (!currentPose.isHeadingErrorLocked()) {
            //  atan2 returns an angle between -180 < theta <= 180
        //TODO: Clean this up if it works
        this.heading = Math.toDegrees(Math.atan2(yError, xError));
        //}
        //  For PID control loop, it is useful to associate a signOfMagnitude to the angle
        //    NEGATIVE ->   -180 < theta <= 0       (yep, zero is negative now)
        //    POSITIVE ->      0 < theta <= 180
        signOfMagnitude = PoseError.getSignOfHeading(this.heading);
        this.calculateErrorControlValue();


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

        this.calculateErrorControlValue();
    }

    public void addToXErrorSum(double error) {this.xErrorSum += error;}
    public void addToYErrorSum(double error) {this.yErrorSum += error;}
    public void addToSpinErrorSum(double error) {this.spinErrorSum += error;}

    private void calculateErrorControlValue() {
        this.errorControlValue = this.magnitude + Math.abs(this.errorSum);
    }


    //***************************************************************88
    //******    STATIC METHODS
    //***************************************************************88

    public static Integer getSignOfHeading(Double inputHeading){
        //  For PID control loop, it is useful to associate a signOfMagnitude to the angle
        //    NEGATIVE ->   -180 < theta <= 0       (yep, zero is negative now)
        //    POSITIVE ->      0 < theta <= 180
        return (inputHeading<=0)? -1 : 1;
    }

}
