package com.company;

public class Robot {
    public static boolean usingComputer = true;

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

        worldXPosition = org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition.getX();
        worldYPosition = org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition.getY();

        worldAngle_rad = org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition.getOrientationRad();





//        SpeedOmeter.yDistTraveled += ySpeed * elapsedTime * 1000;
//        SpeedOmeter.xDistTraveled += xSpeed * elapsedTime * 1000;

//        SpeedOmeter.update();






    }
}
