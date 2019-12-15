package org.firstinspires.ftc.teamcode;

public class Bridge {

    //***********************************************************
    //*****     Class Variables
    //***********************************************************

    private double innerLaneY;       //for blue alliance
    private double outerLaneY;       //for blue alliance
    private double quarrySideX;     //for both alliances
    private double foundationSideX;  //for both alliances


    //***********************************************************
    //*****     Enumerations
    //***********************************************************
    public enum BridgeLane{
        INNER_LANE,
        OUTER_LANE
    }

    //***********************************************************
    //*****     CONSTRUCTORS
    //***********************************************************

    public Bridge(){
        innerLaneY = 38.0;       //for blue alliance
        outerLaneY = 57.0;       //for blue alliance
        quarrySideX = -14.0;     //for both alliances
        foundationSideX = 6.0;
    }

    public Bridge(eBotsAuton2019.Alliance alliance){
        this();
        if(alliance == eBotsAuton2019.Alliance.RED){
            innerLaneY = -innerLaneY;
            outerLaneY = -outerLaneY;
        }
    }

    //***********************************************************
    //*****     Class Methods
    //***********************************************************
    public Pose getQuarrySideStagingPose(BridgeLane bridgeLane){
        double yCoord = (bridgeLane == BridgeLane.INNER_LANE) ? innerLaneY : outerLaneY;
        return new Pose(quarrySideX, yCoord , 0.0);
    }

    public Pose getQuarrySideStagingPose(BridgeLane bridgeLane, double headingOverride){
        Pose pose = getQuarrySideStagingPose(bridgeLane);
        pose.setHeading(headingOverride);
        return pose;
    }

    public Pose getFoundationSideStagingPose(BridgeLane bridgeLane){
        double yCoord = (bridgeLane == BridgeLane.INNER_LANE) ? innerLaneY : outerLaneY;
        return new Pose(foundationSideX, yCoord , 0.0);
    }

    public Pose getFoundationSideStagingPose(BridgeLane bridgeLane, double headingOverride){
        Pose pose = getFoundationSideStagingPose(bridgeLane);
        pose.setHeading(headingOverride);
        return pose;
    }


    public Pose getParkPose(BridgeLane bridgeLane){
        double yCoord = (bridgeLane == BridgeLane.INNER_LANE) ? innerLaneY : outerLaneY;
        return new Pose(0.0, yCoord , 0.0);
    }



}
