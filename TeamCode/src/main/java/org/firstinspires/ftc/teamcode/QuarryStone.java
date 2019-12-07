package org.firstinspires.ftc.teamcode;


import android.util.Log;

import java.util.ArrayList;
import java.util.ListIterator;

import static java.lang.String.format;

public class QuarryStone {

    /***************************************************************
     //******    STATIC VARIABLES
     //****************************************************************/
    private static ArrayList<QuarryStone> quarryStones = new ArrayList<>();
    private static ArrayList<QuarryStone> skyStones = new ArrayList<>();
    private static long observationTimeWindow = 10000L;

     /****************************************************************
     //******    CLASS VARIABLES
     //***************************************************************/

    private Integer numObservations;
    private Integer numIdSkyStone;
    private StoneLocation stoneLocation;
    private ArrayList<TimeStampedObservation> tsObservations;
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
        this.tsObservations = new ArrayList<>();
    }

    public QuarryStone(StoneLocation location, eBotsAuton2019.Alliance alliance){
        this();
        this.stoneLocation = location;
        this.x = stoneLocation.xStoneEnum;
        this.y = stoneLocation.yStoneEnum;
        if(alliance == eBotsAuton2019.Alliance.RED){
            this.y = -y;
        }
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
        //when first build quarry, make sure that skystones array is clear
        if (skyStones.size() > 0) skyStones.clear();

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

    public static int getCountObservedStones(){
        /**
         * Report the number of stones that have been observed
         */
        int countObserved = 0;
        for(QuarryStone stone: quarryStones) {
            if (stone.numObservations > 0)
                countObserved++;
        }
        return countObserved;
    }

    public static ArrayList<QuarryStone> getObservedStones(){
        /**
         * Return a list of stones which have TSObservations recorded
         */
        ArrayList<QuarryStone> observedStones = new ArrayList<>();
        for(QuarryStone stone: quarryStones) {
            if (stone.numObservations > 0)
                observedStones.add(stone);
        }
        return observedStones;
    }

    public static int getTotalObservations(){
        /**
         * Report the total number of observations for all quarry stones
         */
        int totalObservations = 0;
        for(QuarryStone stone: quarryStones){
            totalObservations += stone.numObservations;
        }
        return totalObservations;
    }

    /**
     * This method is intended to give an in-process view into whether the
     * Skystones have been observed in the quarry
     * @return countSkyStones
     */
    public static int getCurrentCountSkyStones(){
        int countSkyStones = 0;
        for(QuarryStone stone: quarryStones) {
            if (stone.isSkyStone())
                countSkyStones++;
        }
        return countSkyStones;
    }

    /**
     * This gives the size of the skyStones Array
     * It is intended to be accessed after the setSkyStones function
     * has recorded the determined skyStones
     * @return skyStones.size()
     */
    public static int getFoundSkyStoneCount(){
        return skyStones.size();
    }


    /**
     * Returns an array containing the found skyStones
     * @return skyStones
     */
    public static ArrayList<QuarryStone> getSkyStones(){
        return skyStones;
    }

    public static void setSkyStones(StoneLocation observedSkyStoneLocation){
        /**
         * Create the arraylist of skyStones based on the observation of the quarry
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


    /**
     * Remove expired quarry stone observations
     * @param minAllowedTime
     */
    protected static void purgeTSObservationsBefore(long minAllowedTime){
        boolean debugOn = true;
        String logTag = "BTI_purgeTSObs";
        if (debugOn) Log.d(logTag, "Purging old records before " + minAllowedTime);

        //Loop through each quarrystone
        for(QuarryStone stone: quarryStones){
            //  Loop through each time stamped observation within a quarrystone
            if (debugOn) Log.d(logTag, "Quarry Stone: " + stone.getStoneLocation().name() +
                    " began with " + stone.tsObservations.size() + " observations" );
            ListIterator<TimeStampedObservation> li = stone.tsObservations.listIterator();
            while(li.hasNext()){
                TimeStampedObservation tsObs = li.next();
                //  Remove it if the timestamp occurred before the minAllowedTime
                if(tsObs.timeStamp < minAllowedTime){
                    li.remove();
                }
            }

            if (debugOn) Log.d(logTag, "Quarry Stone: " + stone.getStoneLocation().name() +
                    " ended with " + stone.tsObservations.size() + " observations" );
            stone.numObservations = stone.tsObservations.size();
        }
    }

    /***************************************************************
     //******    CLASS METHODS
     //****************************************************************/

    /**
     * Calculates the probability that a certain stone is a SkyStone
     * @return  Double which is:
     *              0 if too few observations
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
     * Calculates the probability that a certain stone is a SkyStone
     * including consideration for time stamp
     * @return  Double which is:
     *              0 if too few observations
     *              Probability of being a Skystone
     */
    public Double probabilitySkyStone(long currentTime){
        boolean debugOn = true;
        String logTag = "BTI_probSkyStone(long)";
        if (debugOn) Log.d(logTag, "calculating time-based probability...");

        double probability;
        long minAllowedTime = currentTime - observationTimeWindow;
        if (debugOn) Log.d(logTag, "currentTime: " + currentTime + " minAllowedTime: " + minAllowedTime);

        purgeTSObservationsBefore(minAllowedTime);

        //  For the time based observations, the number of observations equals the size of the tsObservation array
        if (this.tsObservations.size() < 1){
            probability = 0.0;
            this.numIdSkyStone = 0;
        } else {
            probability = ((double) this.numTSSkyStoneObservations()) / this.tsObservations.size();
        }
        if (debugOn) Log.d(logTag, "COMPLETED time-based probability!");
        if (debugOn) Log.d(logTag, "Probability SkyStone for stone: " + this.getStoneLocation().name() +
                " is " + format("%.2f", probability));
        return probability;
    }

    private int numTSSkyStoneObservations(){
        boolean debugOn = true;
        String logTag = "BTI_numTSSkyStoneObs";
        if (debugOn) Log.d(logTag, "Entering numTSSkyStoneObservations.." );

        int tsSkyStoneObservations = 0;
        for(TimeStampedObservation tsObs: this.tsObservations){
            if (tsObs.observedAsSkystone) tsSkyStoneObservations++;
        }
        this.numIdSkyStone = tsSkyStoneObservations;
        if (debugOn) Log.d(logTag, tsSkyStoneObservations + " of " + this.numObservations + " total obs");
        return tsSkyStoneObservations;
    }

    /**
     * Determines if value is a skyStone
     * @return  Double which is:
     *              Null if too few observations
     *              Probability of being a Skystone
     */

    public Boolean isSkyStone(long currentTime){
        boolean debugOn = true;
        String logTag = "BTI_isSkyStone(long)";
        if (debugOn) Log.d(logTag, "recording time-based skystone observation...");

        Boolean returnValue;
        if (probabilitySkyStone(currentTime) > 0.50){
            returnValue = true;
        } else {
            returnValue = false;
        }
        if (debugOn) Log.d(logTag, "COMPLETED time-based skystone observation1");
        return returnValue;
    }

    public boolean isSkyStoneAlreadyInSkyStonesArray(){
        boolean alreadyInArray = false;
        for (QuarryStone skyStone : QuarryStone.getSkyStones()) {
            if (skyStone.getStoneLocation() == this.getStoneLocation()) {
                alreadyInArray = true;
            }
        }
        return alreadyInArray;
    }

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
        /**
         * Record observation if identified as a SkyStone
         * Record observation (whether or not it is a SkyStone)
         */
        if (isSkyStone){
            this.numIdSkyStone += 1;
        }
        this.numObservations += 1;
    }

    public void recordTSObservation(long timeStamp, boolean isSkyStone){
        /**
         * Record timestamp
         * Record observation if identified as a SkyStone
         * Record observation (whether or not it is a SkyStone)
         */
        boolean debugOn = true;
        String logTag = "BTI_recordTSObs";
        if (debugOn) Log.d(logTag, "Entering recordTSObservation...");
        TimeStampedObservation tsObs = new TimeStampedObservation(timeStamp, isSkyStone);

        if (debugOn) Log.d(logTag, "About to record TSObservation " + tsObs.toString());
        this.tsObservations.add(tsObs);
        if (debugOn) Log.d(logTag, "Observation Added successfully");
        //This reports a boolean as to whether or not the observed stone is a skystone
        //  It also updates the properties:
        //      *numObservations   (Via purgeTSObservationsBefore)
        //      *numIdSkyStone  (via numTSSkyStoneObservations)
        //      These two updates allow for the dynamic property isSkyStone() to compute accurately during
        //      determineSkyStonePattern() at the end of ObserveQuarryDuringInit thread

        if (debugOn) Log.d(logTag, "Recording skyStone observation...");
        this.isSkyStone(timeStamp);
        if (debugOn) Log.d(logTag, "skyStone observation recorded successfully");
    }

    @Override
    public String toString(){
        return "Position " + this.stoneLocation.name() + " (" + this.getX() +
                ", " + this.getY() + ") , SkyStone Probability: "
                + format("%.2f", this.probabilitySkyStone()) + " Observed " + this.numObservations
                + " times";
    }

    /***************************************************************
     //******    Inner Classes
     //****************************************************************/
    class TimeStampedObservation{
        private long timeStamp;
        private boolean observedAsSkystone;

        public TimeStampedObservation(){
        }
        public TimeStampedObservation(long timeStamp, boolean isSkyStone){
            this.timeStamp = timeStamp;
            this.observedAsSkystone = isSkyStone;
        }

        public long getTimeStamp(){return timeStamp;}
        public boolean getObervedAsSkystone(){return this.observedAsSkystone;}

        @Override
        public String toString(){
            return "TSObservation, timeStamp:" + this.timeStamp + " observedAsSkyStone: " + this.observedAsSkystone;
        }
    }

}
