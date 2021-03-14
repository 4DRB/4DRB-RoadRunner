package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class WebcamCalibration extends LinearOpMode
{
    static double ringCount = 0;
    static double average1 = 0;
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
        });
        telemetry.addData("Suntem in cazul",ringCount);
        telemetry.addData("Average",pipeline.avg1);
        telemetry.update();


        waitForStart();



while (opModeIsActive()){
    telemetry.addData("Suntem in cazul",ringCount);
    telemetry.addData("Average",average1);
    telemetry.update();
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

        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 20;

        final int FOUR_RING_THRESHOLD = 150;//150lumina multa//135luminaputina
        final int ONE_RING_THRESHOLD = 135;//135luminamulta//128luminaputina

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
        public volatile RingPosition position = RingPosition.FOUR;

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
                average1 = avg1;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                ringCount=1;
                average1 = avg1;
            }else{
                position = RingPosition.NONE;
                ringCount=0;
                average1 = avg1;
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