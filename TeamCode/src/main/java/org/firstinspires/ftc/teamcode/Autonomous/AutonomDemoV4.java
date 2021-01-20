/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous
public class AutonomDemoV4 extends LinearOpMode
{
    static double ringCount = 0;
    RingDetectingPipeline pipeline;
    private static int intPosition;
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    public double offTLD=3.25;
    public double offBLD=2.25;
    public double offTRD=4;
    public double offBRD=6.5;
    double  power   = 0.3;
    final double TICKS_PER_REV = 537.6 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference 9.6
    double TICKS_PER_CM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    int initDiff,lastDiff,diffDiff;
    Encoder leftEncoder, rightEncoder, frontEncoder;
    OpenCvCamera webCam = null;





    @Override
    public void runOpMode()
    {
// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).



        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new RingDetectingPipeline();
        webCam.setPipeline(new RingDetectingPipeline());
        //starts the stream
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {

            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });FL = hardwareMap.get(DcMotor.class, "leftFront");
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        BL = hardwareMap.get(DcMotor.class, "leftRear");
        BR = hardwareMap.get(DcMotor.class, "rightRear");
//stop and reset
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// this offsets the two motors that are facing the opposite direction.
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");

        telemetry.addData("Suntem in cazul",ringCount);
        telemetry.update();


        waitForStart();




        if(ringCount == 1.0){


            wRelease.setPosition(0);
            encoderDrive(0.5,145,145);
            strafeDrive(0.3,47,-47);
            MultiShotAutonom();
            //arunca gogosile,se duca sa mai ia gogosi
            IntakeAutonom(1);
            encoderDrive(0.3,-30,-30);
            sleep(2700);
            IntakeAutonom(0);
            encoderDrive(0.5,30,30);
            MultiShotAutonom();
            //arunca gogosile,se duce la  patrat 2
            encoderDrive(0.5,100,100);
            strafeDrive(0.4,35,-35);

            wRelease.setPosition(0.4);

            for(int i =1;i<=2000;i++)
            {wRelease.setPosition(0.4);}
            sleep(500);

            encoderDrive(0.5,-70,-70);
            stop();
        }else if(ringCount == 4.0){
            wRelease.setPosition(0);
            encoderDrive(0.5,145,145);
            strafeDrive(0.3,47,-47);
            MultiShotAutonom();
            //arunca gogosile,se duca sa mai ia gogosi
            IntakeAutonom(1);
            encoderDrive(0.3,-30,-30);
            sleep(1500);
            encoderDrive(0.3,-11,-11);
            sleep(1500);
            encoderDrive(0.3,-11,-11);
            sleep(2700);
            IntakeAutonom(0);
            encoderDrive(0.5,52,52);
            MultiShotAutonom();
            //arunca gogosile,se duce la  patrat 3
            encoderDrive(0.5,140,143);
            strafeDrive(0.4,-25,25);

            encoderDrive(0.5,30,-30);
            wRelease.setPosition(0.4);

            for(int i =1;i<=2000;i++)
            {wRelease.setPosition(0.4);}
            sleep(500);
            encoderDrive(0.5,-30,30);
            encoderDrive(0.5,-100,-103);




            //drive somewhere else
            stop();
        }else{
            //drive to a different spot
            Shooter.setPosition(0.1);
            Shooter.setPosition(0);
            wRelease.setPosition(0);
            sleep(100);
            encoderDrive(0.5,190,190);
            wRelease.setPosition(0.4);
            strafeDrive(0.5,48,-48);

            encoderDrive(0.5,-43,-43);

            MultiShotAutonom();
            strafeDrive(0.5,15,-15);
            sleep(100);
            encoderDrive(0.5,117.25,-117.25);
            encoderDrive(0.5,73,73);

            CremalieraAutonom(-0.8);
            ClampAutonom(0);
            sleep(500);
            GlisieraAutonom(200);



            encoderDrive(0.5,-135,-135);
            encoderDrive(0.5,58.6,-58.6);
            encoderDrive(0.5,15,15);

            ClampAutonom(0.3);
            CremalieraAutonom(0.8);

            strafeDrive(0.5,-10,10);
            encoderDrive(0.6,-10,-10);
            stop();
        }

    }


    public void encoderDrive(double speed, double leftInches, double rightInches){

// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converdets it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));
        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",FL.getCurrentPosition());
            telemetry.update();
            initDiff=frontEncoder.getCurrentPosition()-leftEncoder.getCurrentPosition();
        }


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






    }
    public void odometryFwdDrive(){

    }
    public void strafeDrive(double speed, double leftInches, double rightInches){
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BL.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        // this flips the diagonals which allows the robot to strafe
        FL.setTargetPosition(-newLeftFrontTarget);
        FR.setTargetPosition(-newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));


        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",FL.getCurrentPosition());
            telemetry.update();
        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void SingleShotAutonom(){
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1000);
        Shooter.setPosition(0.3);
        sleep(300);
        Shooter.setPosition(0);
        sleep(100);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }

    public void MultiShotAutonom(){
        double power = -0.95;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1100);
        Shooter.setPosition(0.3);
        sleep(200);
        Shooter.setPosition(0);
        sleep(300);
        Shooter.setPosition(0.3);
        sleep(200);
        Shooter.setPosition(0);
        sleep(300);
        Shooter.setPosition(0.3);
        sleep(200);
        Shooter.setPosition(0);
        sleep(300);
        Shooter.setPosition(0.3);
        sleep(200);
        Shooter.setPosition(0);
        // sleep(300);
        Launcher1.setPower(0);
        Launcher2.setPower(0);



    }
    public void IntakeAutonom(double power){
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class,"leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        InTake.setPower(power);
    }
    public void ClampAutonom(double position) {
        Servo clamp = hardwareMap.get(Servo.class, "SR_CLAMP");


        clamp.setPosition(position);
        sleep(500);
        //telemetry.addData("Servo Position", clamp.getPosition());
        //telemetry.addData("Status", "Running");
    }
    public void CremalieraAutonom(double viteza) {      //     - scoate , + baga
        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel mag_crm = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        mag_crm.setMode(DigitalChannel.Mode.INPUT);

        cremaliera_Servo.setPower(viteza);

        sleep(500);

        while (mag_crm.getState())
        {
            cremaliera_Servo.setPower(viteza);
            if (!mag_crm.getState())
            {
                cremaliera_Servo.setPower(0);
                break;
            }
        }
        if (!mag_crm.getState())
        {
            cremaliera_Servo.setPower(0);
        }



    }
    public void GlisieraAutonom(long sleep) {

        DcMotor glisiera = hardwareMap.get(DcMotorEx.class, "GLS");
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DigitalChannel glsMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");


        glisiera.setPower(1);
        sleep(sleep);
        glisiera.setPower(0);
    }


    public void move(double x, double y, double rot, double cmdistance){
        //do some sort of distance conversion
        double distance;
        distance = cmdistance * TICKS_PER_CM;
        if(FL.getCurrentPosition() < FL.getCurrentPosition() + distance && FR.getCurrentPosition() < FR.getCurrentPosition() + distance
                && BR.getCurrentPosition() < BR.getCurrentPosition() + distance && BL.getCurrentPosition() < BL.getCurrentPosition() + distance)
        {
            //drive at a power
            setPower(x,y,rot);
        }
        else{
            setPower(0.0,0.0,0.0);
        }
        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",FL.getCurrentPosition());
            telemetry.update();
        }
        setPower(0.0,0.0,0.0);
    }

    public void setPower(double x, double y, double rot){
        double frontLeftMotorPower = y - x - rot;
        double frontRightMotorPower = y + x + rot;
        double backLeftMotorPower = y + x - rot;
        double backRightMotorPower = y - x + rot;

        double motorPowers[] = {Math.abs(frontLeftMotorPower),
                Math.abs(backRightMotorPower),
                Math.abs(backLeftMotorPower),
                Math.abs(frontRightMotorPower)};

        Arrays.sort(motorPowers);

        if(motorPowers[3] != 0){
            frontLeftMotorPower /= motorPowers[3];
            frontRightMotorPower /= motorPowers[3];
            backRightMotorPower /= motorPowers[3];
            backLeftMotorPower /= motorPowers[3];
        }
    }











    public static class RingDetectingPipeline extends OpenCvPipeline
    {
        public static int getPosition() {
            return intPosition;
        }

        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70,170);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;


        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
                ringCount=4;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                ringCount=1;
            }else{
                position = RingPosition.NONE;
                ringCount=0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }


    }
    public void WRealeaseLeftAutonom(double position) {
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");

        wRelease.setPosition(position);

        for(int i =1;i<=2000;i++)
        {wRelease.setPosition(position);}

        telemetry.addData("Servo Position", wRelease.getPosition());
        telemetry.update();
    }



}