package org.firstinspires.ftc.teamcode;


import android.util.Log;

import java.util.ArrayList;

import static java.lang.String.format;

public class QuarryStone {

    /***************************************************************
     //******    STATIC VARIABLES
     //****************************************************************/
    private static ArrayList<QuarryStone> quarryStones = new ArrayList<>();


     /****************************************************************
     //******    CLASS VARIABLES
     //***************************************************************/

    private Integer numObservations;
    private Integer numIdSkyStone;
    private StoneLocation stoneLocation;


    /***************************************************************
     //******    ENUMERATIONS
     //***************************************************************/

    public enum StoneLocation{
        ZERO (-68.0, 22.0),
        ONE (-60.0, 22.0),
        TWO (-52.0, 22.0),
        THREE (-44.0, 22.0),
        FOUR (-36.0, 22.0),
        FIVE (-28.0, 22.0);

        private double xStone;
        private double yStone;


        StoneLocation(Double x, Double y){
            this.xStone = x;
            this.yStone = y;
        }

        //**        ENUM GETTERS & Setters         ***************
        public double getXStone() {return this.xStone;}
        public double getYStone() {return this.yStone;}
        public void setXStone(Double xInput) {this.xStone = xInput;}
        public void setYStone(Double yInput) {this.yStone = yInput;}

        public static StoneLocation getStoneLocation(int location){
            StoneLocation returnLocation;
            if (location == 0) returnLocation = StoneLocation.ZERO;
            else if (location == 1) returnLocation = StoneLocation.ONE;
            else if (location == 2) returnLocation = StoneLocation.TWO;
            else if (location == 3) returnLocation = StoneLocation.THREE;
            else if (location == 4) returnLocation = StoneLocation.FOUR;
            else returnLocation = StoneLocation.FIVE;

            return returnLocation;
        }
    }
    /***************************************************************
     //******    CONSTRUCTORS
     //***************************************************************/

    public QuarryStone(){
        this.stoneLocation = null;
        this.numObservations = 0;
        this.numIdSkyStone = 0;
    }

    public QuarryStone(StoneLocation location, eBotsAuton2019.Alliance alliance){
        this.stoneLocation = location;
        if(alliance == eBotsAuton2019.Alliance.RED){
            this.stoneLocation.setYStone(-this.stoneLocation.getYStone());
        }
        this.numObservations = 0;
        this.numIdSkyStone = 0;
        quarryStones.add(this);
    }

    /***************************************************************
     //******    GETTERS
     //****************************************************************/
    public static ArrayList<QuarryStone> getQuarryStones(){return quarryStones;}
    public StoneLocation getStoneLocation(){return this.stoneLocation;}



    /***************************************************************
     //******    STATIC METHODS
     //****************************************************************/
    public static void constructQuarry(eBotsAuton2019.Alliance alliance){
        Log.d ("constructQuarry", "Beginning to construct Quarry");
        if (quarryStones.size() > 0) quarryStones.clear();            //Make sure it's empty (from earlier issue with encoders)

        //Loop through the values of the enumerations and create a stone for each one
        for (StoneLocation loc: StoneLocation.values()){
            QuarryStone stone= new QuarryStone(loc, alliance);
            //  Stones are automatically added to the list during instantiation
            Log.d("constructQuarry", "Stone added for position --> " + loc.toString());
        }
        String debugMessage = (Integer)(quarryStones.size()) + " Stones Added";
        Log.d ("constructQuarry", debugMessage);

    }

    public static QuarryStone getQuarryStone(StoneLocation location){
        QuarryStone returnStone = null;
        for (QuarryStone stone: quarryStones){
            if (stone.stoneLocation == location){
                returnStone = stone;
                break;
            }
        }
        return returnStone;
    }


    public static ArrayList<QuarryStone> getSkyStones(){

        ArrayList<QuarryStone> skyStones = new ArrayList<>();   //Create list to contain SkyStones
        if (skyStones.size() > 0) skyStones.clear();            //Make sure it's empty (from earlier issue with encoders)
        for (QuarryStone stone: quarryStones){
            if (stone.isSkyStone() == true){
                skyStones.add(stone);
            }
        }
        //Make sure that only returning 2 skyStones
        if (skyStones.size() > 2) {
            //  First compare probability, then compare num observations

        }
        return skyStones;
    }

    public static int getFoundSkyStoneCount(){
        return getSkyStones().size();
    }


    /***************************************************************
     //******    CLASS METHODS
     //****************************************************************/

    /**
     * Calculates the probability that a certain stone is a SkyStone
     * @return  Double which is:
     *              Null if too few observations
     *              Probability of being a Skystone
     */
    public Double probabilitySkyStone(){
        Double probability;
        if (this.numObservations < 10){
            probability = 0.0;
        } else {
            probability = ((double) numIdSkyStone) / numObservations;
        }
        return probability;
    }
    /**
     * Determines if value is a skyStone
     * @return  Double which is:
     *              Null if too few observations
     *              Probability of being a Skystone
     */

    public Boolean isSkyStone(){
        Boolean returnValue;
        if (probabilitySkyStone() > 0.8){
            returnValue = true;
        } else {
            returnValue = false;
        }
        return returnValue;
    }

    public void recordObservation(Boolean isSkyStone){
        if (isSkyStone){
            this.numIdSkyStone += 1;
        }
        this.numObservations += 1;
    }

    @Override
    public String toString(){
        return "Position " + this.stoneLocation.name() + " , SkyStone Probability: "
                + format("%.2f", this.probabilitySkyStone()) + " Observed " + this.numObservations
                + " times";
    }

}
