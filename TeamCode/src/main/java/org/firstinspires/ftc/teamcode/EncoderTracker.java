package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class EncoderTracker {

    private DcMotor motor;
    private RobotOrientation robotOrientation;
    private static Double clicksPerInch = 434.82;
    private Integer currentClicks;
    private static ArrayList<EncoderTracker> encoders = new ArrayList<>();

    public EncoderTracker (DcMotor motor, RobotOrientation robotOrientation){
        this.motor = motor;
        this.robotOrientation = robotOrientation;
        this.currentClicks = 0;
        encoders.add(this);
    }


    public enum RobotOrientation{
        FORWARD,
        LATERAL
    }

    public static Pose getNewPose (Pose previousPose){
        Integer newClicks;
        Double newX = previousPose.getX();
        Double newY = previousPose.getY();
        Double encoderAngle;
        Double totalDistance;

        // TODO: 10/13/2019 Add consideration for rotation

        //  Loop through the encoders to figure out how much
        for(EncoderTracker e: encoders){
            //Calculate distance traveled for encoder
            newClicks = e.motor.getCurrentPosition();
            totalDistance = (newClicks - e.currentClicks) * clicksPerInch;

            //Consider the placement of the encoder on the robot
            //This assumes that when robot front oriented with positive X direction
            //  that clicks increase with x+ and y+ field oriented vectors
            encoderAngle = previousPose.getHeading();

            //Lateral encoder must shift angle to calculate effect on X and Y coordinates
            if(e.robotOrientation == RobotOrientation.LATERAL) {
                encoderAngle += 90;
            }

            Double deltaX = totalDistance * Math.cos(Math.toRadians(encoderAngle));
            Double deltaY = totalDistance * Math.sin(Math.toRadians(encoderAngle));

            newX += deltaX;
            newY += deltaY;
        }
        Pose newPose = new Pose(newX, newY, previousPose.getHeading());
        return newPose;
    }

}
