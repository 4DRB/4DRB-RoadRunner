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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class wReleaseTesting extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    private static int intPosition;
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;
    public double offTLD=3.25;
    public double offBLD=2.25;
    public double offTRD=4;
    public double offBRD=6.5;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    double  power   = 0.3;




    @Override
    public void runOpMode() throws InterruptedException {
// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*TleftDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();



            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            if (intPosition == 4)
            {
                telemetry.addData("It's",4 );
                telemetry.update();
                Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");

                wRelease.setPosition(0.4);
                Thread.sleep(1000);

                telemetry.addData("Servo Position", wRelease.getPosition());
                telemetry.update();


            }
            if (intPosition == 1)
            {
                telemetry.addData("It's",1 );
                telemetry.update();
                WRealeaseLeftAutonom(0.0);

            }
            if (intPosition == 0)
            {


                WRealeaseLeftAutonom(0.4);


            }// Don't burn CPU cycles busy-looping in this sample


    }
    public void WRealeaseLeftAutonom(double position) {
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");

        wRelease.setPosition(position);

        for(int i =1;i<=2000;i++)
        {wRelease.setPosition(position);}

        telemetry.addData("Servo Position", wRelease.getPosition());
        telemetry.update();
    }
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

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
                intPosition=4;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                intPosition=1;
            }else{
                position = RingPosition.NONE;
                intPosition=0;
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

    public void Forward(int target  , double power) {

        TleftDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrightDrive.setPower(-power + power*-offTRD/100);
        BrightDrive.setPower(-power + power*-offBRD/100);
        TleftDrive.setPower(-power + power*-offTLD/100);
        BleftDrive.setPower(-power + power*-offBLD/100);

        TrightDrive.setTargetPosition(target);
        BrightDrive.setTargetPosition(target);
        TleftDrive.setTargetPosition(target);
        BleftDrive.setTargetPosition(target);

        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currPos = TrightDrive.getCurrentPosition();
        int targetPos = TrightDrive.getTargetPosition();
        while(currPos<targetPos){
            currPos = TrightDrive.getCurrentPosition();
            targetPos = TrightDrive.getTargetPosition();
        if (currPos>=targetPos){
            break;
        }
        }
        TleftDrive.close();
        TrightDrive.close();
        BleftDrive.close();
        BrightDrive.close();
    }
    public  void Back(int target  , double power) {
        TleftDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TleftDrive.setPower(power);
        BleftDrive.setPower(power);
        TrightDrive.setPower(power);
        BrightDrive.setPower(power);

        TrightDrive.setTargetPosition(target);
        BrightDrive.setTargetPosition(target);
        TleftDrive.setTargetPosition(target);
        BleftDrive.setTargetPosition(target);

        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int currPos = TrightDrive.getCurrentPosition();
        int targetPos = TrightDrive.getTargetPosition();
        while(currPos<targetPos){
            currPos = TrightDrive.getCurrentPosition();
            targetPos = TrightDrive.getTargetPosition();
            if (currPos>=targetPos){
                break;
            }
        }
        TleftDrive.close();
        TrightDrive.close();
        BleftDrive.close();
        BrightDrive.close();
    }
    public void Left(int target  , double power) {
        TleftDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TleftDrive.setPower(power + power*offTLD/100);
        BleftDrive.setPower(-power + power* -offBLD/100);
        TrightDrive.setPower(-power + power* -offTRD/100);
        BrightDrive.setPower(power + power* offBRD/100);

        TrightDrive.setTargetPosition(target);
        BrightDrive.setTargetPosition(target);
        TleftDrive.setTargetPosition(target);
        BleftDrive.setTargetPosition(target);

        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int currPos = TrightDrive.getCurrentPosition();
        int targetPos = TrightDrive.getTargetPosition();
        while(currPos<targetPos){
            currPos = TrightDrive.getCurrentPosition();
            targetPos = TrightDrive.getTargetPosition();
            if (currPos>=targetPos){
                break;
            }
        }
        TleftDrive.close();
        TrightDrive.close();
        BleftDrive.close();
        BrightDrive.close();
    }
    public void Right(int target  , double power) {
        TleftDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        TleftDrive.setPower(-power + power*-offTLD/100);
        BleftDrive.setPower(power + power*offBLD/100);
        TrightDrive.setPower(power + power*offTRD/100);
        BrightDrive.setPower(-power + power*-offBRD/100);

        TrightDrive.setTargetPosition(target);
        BrightDrive.setTargetPosition(target);
        TleftDrive.setTargetPosition(target);
        BleftDrive.setTargetPosition(target);

        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int currPos = TrightDrive.getCurrentPosition();
        int targetPos = TrightDrive.getTargetPosition();
        while(currPos<targetPos){
            currPos = TrightDrive.getCurrentPosition();
            targetPos = TrightDrive.getTargetPosition();
            if (currPos>=targetPos){
                break;
            }
        }
        TleftDrive.close();
        TrightDrive.close();
        BleftDrive.close();
        BrightDrive.close();
    }
    public void Stop(){
        TleftDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrightDrive.setPower(0);
        BrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BleftDrive.setPower(0);
    }


}