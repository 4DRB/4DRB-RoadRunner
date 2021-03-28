package com.company;


import android.graphics.Point;


import treamcode.MyOpMode;


public class Robot {

    Point SP = new Point(0,0);
    public static boolean usingComputer = true;
MyOpMode coords = new MyOpMode();

    /**
     * Creates a robot simulation
     */
    public Robot(){
        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle_rad = Math.toRadians(90);
    }

    //the actual speed the robot is moving
    public static double xSpeed = 0;
    public static double ySpeed = 0;
    public static double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;

    static final double TRACKWIDTH = 17.7;
    static final double WHEEL_DIAMETER = 6.0*2.54;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 2.4;


    public double getXPos(){
        return worldXPosition;
    }

    public double getYPos(){
        return worldYPosition;
    }


    public double getWorldAngle_rad() {
        return worldAngle_rad;
    }


    //last update time
    private long lastUpdateTime = 0;

    /**
     * Calculates the change in position of the robot
     */
    public void update(){
        //get the current time


        //increment te positions
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / 8192;
        worldXPosition = coords.WorldX();
        worldYPosition = coords.WorldY();

        worldAngle_rad = coords.WorldAngle();





//        SpeedOmeter.yDistTraveled += ySpeed * elapsedTime * 1000;
//        SpeedOmeter.xDistTraveled += xSpeed * elapsedTime * 1000;

//        SpeedOmeter.update();






    }
}
