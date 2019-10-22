package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

public class StopWatch {
    private Long startTime;

    public StopWatch(){
        startTime = 0L;
    }

    public void startTimer(){
        startTime = System.currentTimeMillis();  //current time in milliseconds
    }

    public Long getElapsedTimeMillis(){
        return startTime == 0 ? 0L : System.currentTimeMillis() - startTime;
    }

    public Double getElapsedTimeSeconds(){
        return startTime == 0 ? 0.0 :(double) ((System.currentTimeMillis() - startTime) / 1000.0);
    }

    @Override
    public String toString(){
        return format("%.2", getElapsedTimeSeconds()) + " seconds";
    }

    public String toString(Integer loopCount){
        Double frequency = loopCount / getElapsedTimeSeconds();
        return loopCount + " loops in " + getElapsedTimeMillis() + " ms or " + format("%.2f", frequency) + " Hz";
    }

}

