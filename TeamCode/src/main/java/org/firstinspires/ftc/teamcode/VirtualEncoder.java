package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static java.lang.String.format;
import static org.firstinspires.ftc.teamcode.EncoderTracker.*;

public class VirtualEncoder {
    private Integer clicks;

    private static final Double clicksPerInch = 434.82;
    private static final Double topSpeed = 50.0;  // inches/s
    private static final double angularTopSpeed = 276.92;  //  degrees / s
    private static final double virSpinRadius = 6.0;   //  inches from spin center
    private static String debugTag = "BTI_VirtualEncoder";

    public VirtualEncoder(){
        clicks = 0;
    }

    public void simulateLoopOutput(Double distance, Double encoderAngleRad, RobotOrientation robotOrientation){
        boolean debugOn = false;
        String logTag = "BTI_simLoopOut(Trans)";
        double distanceComponent;
        if(robotOrientation == RobotOrientation.FORWARD){
            distanceComponent = distance * Math.cos(encoderAngleRad);
        } else {
            distanceComponent = distance * Math.sin(encoderAngleRad);
        }
        int newClicksToBeAdded = (int) Math.round(distanceComponent * clicksPerInch);
        this.clicks += newClicksToBeAdded;
        double encoderAngleDeg = Math.toDegrees(encoderAngleRad);
        if (debugOn) {
            Log.d(logTag, "Added " + newClicksToBeAdded + " to " + robotOrientation.name() + " encoder");
            Log.d(debugTag, format("%.3f", distance) + " in total, " + format("%.3f",distanceComponent) +
                    "in component for " + robotOrientation.name() + " encoder");
        }
    }

    public void simulateLoopOutput (double distance, double encoderAngleRad, RobotOrientation ro, double spinDistance){
        boolean debugOn = false;
        String logTag = "BTI_simulateLoopOutput";
        //  Start with translation component
        simulateLoopOutput(distance, encoderAngleRad, ro);

        //Then apply rotation
        int spinClicks = (int) Math.round(spinDistance * clicksPerInch);
        this.clicks += spinClicks;

        double encoderAngleDeg = Math.toDegrees(encoderAngleRad);

        if (debugOn) Log.d(logTag, spinClicks + " spin clicks applied to "
                + ro.name() + " encoder");

    }

    public Integer getClicks(){
        return this.clicks;
    }

    public static Double calculateSimulatedDistance(double driveSignal, long timeStepMillis){
        double actualSpeed = driveSignal * topSpeed;
        double distance = actualSpeed * (timeStepMillis / 1000.0);
        return distance;
    }

    public static Double calculateSimulatedRotation(double spinSignal, long timeStepMillis){
        boolean debugOn = false;
        String logTag = "BTI_calcSimRot";
        double actualAngularSpeed = spinSignal * angularTopSpeed;
        double rotationAngle = actualAngularSpeed * (timeStepMillis / 1000.0);
        double rotationDistance = virSpinRadius * Math.toRadians(rotationAngle);
        if (debugOn) Log.d(logTag, "With spin signal " + format("%.2f", spinSignal) +
                " rotation distance of " + format("%.2f", rotationDistance) + " output" +
                " which equates to an angle of " + format("%.2f", rotationAngle));
        return rotationDistance;
    }
}
