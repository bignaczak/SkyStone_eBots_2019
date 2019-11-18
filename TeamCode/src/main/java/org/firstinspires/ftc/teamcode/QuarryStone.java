package org.firstinspires.ftc.teamcode;


import android.util.Log;

import java.util.ArrayList;

import static java.lang.String.format;

public class QuarryStone {

    /***************************************************************
     //******    STATIC VARIABLES
     //****************************************************************/
    private static ArrayList<QuarryStone> quarryStones = new ArrayList<>();
    private static ArrayList<QuarryStone> skyStones = new ArrayList<>();

     /****************************************************************
     //******    CLASS VARIABLES
     //***************************************************************/

    private Integer numObservations;
    private Integer numIdSkyStone;
    private StoneLocation stoneLocation;
    private double x;
    private double y;

    /***************************************************************
     //******    ENUMERATIONS
     //***************************************************************/

    public enum StoneLocation{
        ZERO (-66.0, 22.0),
        ONE (-58.0, 22.0),
        TWO (-50.0, 22.0),
        THREE (-42.0, 22.0),
        FOUR (-34.0, 22.0),
        FIVE (-26.0, 22.0);

        private double xStoneEnum;
        private double yStoneEnum;


        StoneLocation(Double x, Double y){
            this.xStoneEnum = x;
            this.yStoneEnum = y;
        }

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
        this.x = stoneLocation.xStoneEnum;
        this.y = stoneLocation.yStoneEnum;
        if(alliance == eBotsAuton2019.Alliance.RED){
            this.y = -y;
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
    public double getX(){return this.x;}
    public double getY(){return this.y;}
    public void setY(double yIn){this.y = yIn;}


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

    public static int getCountObserved(){
        int countObserved = 0;
        for(QuarryStone stone: quarryStones) {
            if (stone.numObservations > 3)
                countObserved++;
        }
        return countObserved;
    }

    public static int getCountSkyStones(){
        int countSkyStones = 0;
        for(QuarryStone stone: quarryStones) {
            if (stone.isSkyStone())
                countSkyStones++;
        }
        return countSkyStones;
    }


    public static ArrayList<QuarryStone> getSkyStones(){
        return skyStones;
    }

    public static void setSkyStones(StoneLocation observedSkyStoneLocation){
        /**
         * Create the arraylist of skyStones based on the obsevation of the quarry
         */
        String logTag = "BTI_setSkyStones";
        Log.d(logTag, "Preparing to write skystones...");
        QuarryStone observedSkyStone = getQuarryStone(observedSkyStoneLocation);
        Log.d(logTag, "Observed stone " + observedSkyStone.toString());
        QuarryStone otherSkyStone;

        //  Empty the array if anything exists
        if (skyStones.size()>0) skyStones.clear();

        if(observedSkyStoneLocation == StoneLocation.ZERO) {
            otherSkyStone = getQuarryStone(StoneLocation.THREE);
        } else if (observedSkyStoneLocation == StoneLocation.ONE){
            otherSkyStone = getQuarryStone(StoneLocation.FOUR);
        } else if (observedSkyStoneLocation == StoneLocation.TWO) {
            otherSkyStone = getQuarryStone(StoneLocation.FIVE);
        } else if (observedSkyStoneLocation == StoneLocation.THREE){
            otherSkyStone = getQuarryStone(StoneLocation.ZERO);
        } else if (observedSkyStoneLocation == StoneLocation.FOUR){
            otherSkyStone = getQuarryStone(StoneLocation.ONE);
        } else {
            otherSkyStone = getQuarryStone(StoneLocation.TWO);
        }
        Log.d(logTag, "Other stone " + otherSkyStone.toString());

        // Add the observed Skystone to the array list
        skyStones.add(observedSkyStone);

        //  Add the other skystone according to the recognized pattern
        skyStones.add(otherSkyStone);

        Log.d(logTag, "...SkyStones written");

    }

    public static int getFoundSkyStoneCount(){
        return skyStones.size();
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
        if (this.numObservations < 1){
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
        if (probabilitySkyStone() > 0.5){
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
        return "Position " + this.stoneLocation.name() + " (" + this.getX() +
                ", " + this.getY() + ") , SkyStone Probability: "
                + format("%.2f", this.probabilitySkyStone()) + " Observed " + this.numObservations
                + " times";
    }

}
