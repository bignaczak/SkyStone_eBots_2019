package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static java.lang.String.format;

public class VirtualEncoder {
    private Integer clicks;

    private static final Double clicksPerInch = 434.82;
    private static final Double topSpeed = 50.0;  // inches/s
    private static String debugTag = "BTI_VirtualEncoder";

    public VirtualEncoder(){
        clicks = 0;
    }

    public void simulateLoopOutput (Double distance, Double encoderAngleRad){

        //Integer signOfTravel = PoseError.getSignOfHeading(currentPose.getHeading());

        Double distanceComponent = distance * Math.cos(encoderAngleRad);
        clicks += (int) Math.round(distanceComponent * clicksPerInch);
        Double encoderAngleDeg = Math.toDegrees(encoderAngleRad);
        Log.d(debugTag, format("%.3f", distance) + " in total, " + format("%.3f",distanceComponent) +
                "in component for " + format("%.1f",encoderAngleDeg) + " encoder");
    }

    public Integer getClicks(){
        return this.clicks;
    }

    public static Double calculateSimulatedDistance(Double driveSignal, Long timeStepMillis){
        Double actualSpeed = driveSignal * topSpeed;
        Double distance = actualSpeed * (timeStepMillis / 1000.0);
        return distance;
    }
}
