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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeleOp.SuperSuperDance;
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

//import org.firstinspires.ftc.teamcode.OdometryAutonomous.OdometryGlobalCoordinatePosition;

@Autonomous
public class AutonomDemoV9 extends LinearOpMode {
    static double ringCount = 0;
    RingDetectingPipeline pipeline;
    private static int intPosition;
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    public double offTLD = 3.25;
    public double offBLD = 2.25;
    public double offTRD = 4;
    public double offBRD = 6.5;
    double power = 0.3;
    final double TICKS_PER_REV = 537.6;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference 9.6
    double TICKS_PER_CM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    final double TICKS_PER_REV_ENC = 8192;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION_ENC = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM_ENC = 6;     // For figuring circumference 9.6
    double TICKS_PER_CM_ENC = 434;//(TICKS_PER_REV_ENC * DRIVE_GEAR_REDUCTION_ENC) / (WHEEL_DIAMETER_CM_ENC * 3.1415);

    int initDiff, lastDiff, diffDiff;
    Encoder leftEncoder, rightEncoder, frontEncoder;
    OpenCvCamera webCam = null;
    Servo Trigger;
    Servo Shooter;
    private Orientation angles;
    private BNO055IMU imu;
    Servo wRelease;
    Servo finger;
    DcMotor Arm;
    public DistanceSensor sensorRange;
    Rev2mDistanceSensor sensorDistance;
    double distance2;
    double stop = 0;
    @Override
    public void runOpMode() {
// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Trigger = hardwareMap.get(Servo.class, "SR_TRIGGER");
        //Shooter.setDirection(Servo.Direction.REVERSE);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        //rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        //frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));
        //Trigger = hardwareMap.get(Servo.class, "SR_TRIGGER");
        //Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");

        //rightEncoder.setDirection(Encoder.Direction.REVERSE);
        //leftEncoder.setDirection(Encoder.Direction.REVERSE);

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

        //starts the stream

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
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
        Arm = hardwareMap.get(DcMotor.class, "ARM");
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        finger = hardwareMap.get(Servo.class, "SR_FINGER");
        wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");
        //sleep(1000);
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorDistance = (Rev2mDistanceSensor) sensorRange;

        DriveThread TeleThread = new DriveThread();

        waitForStart();
        TeleThread.start();
sleep(100);

        //normalDrive(0.6,138,133.5);
        //normalGyroDrive(0.5,-3,0.5);
        /*normalDrive(0.45,145,145);
        InterContinentalBallisticMissleFalling();

*/      /*stop =1;
        sleep(100);
        stop = 0;
        fingerAutonom(0);
*/

            if(ringCount == 1.0){

                wRelease.setPosition(0);
                fingerAutonom(0.5);
                normalDrive(0.7, 155, 155);

                //InterContinentalBallisticMissleShooters();
                //normalGyroDrive(0.25, 0, 0.3);
                //normalDrive(0.45, 40, 40);
                normalstrafeDrive(0.6, -58, 58);
                //normalGyroDrive(0.3, 0, 0.5);
                Katyusha();
                Trigger(0.35);
                Intake(-1);
                normalDrive(0.6, -45, -45);
                normalDrive(0.5, 45, 45);
                Intake(0);
                //normalGyroDrive(0.3, 0, 0.5);
                InterContinentalBallisticMissleSingle(3);

                normalDrive(1, 43, 43);
                normalDrive(1, 55, -55);

                wRelease(0.4);
                normalstrafeDrive(1, 10, -10);
                normalDrive(1, -52, 52);
                normalDrive(1, -100, -100);
                normalstrafeDrive(1, -85, 85);
                normalstrafeDrive(0.4, 15, -15);
                armAutonom(530);
                fingerAutonom(0);
                //sleep(100);
                normalDistanceDrive(0.25, 60, 22.5);
                sleep(200);
                fingerAutonom(1);
                sleep(200);
                armAutonom(150);
                //sleep(100);
                stop = 1;
                sleep(100);
                normalstrafeDrive(1, 90, -90);
                normalDrive(1, 121, -121);
                sleep(100);
                normalDrive(1, -100, -100);
                stop = 0;
                ///sleep(100);
                armAutonom(320);
                sleep(100);
                fingerAutonom(0);
                sleep(100);
                normalDrive(1, 10, 10);
            stop();
        } else
            if(ringCount == 4.0){
                wRelease.setPosition(0);
                fingerAutonom(0.5);
                normalDrive(0.7, 155, 155);

                //InterContinentalBallisticMissleShooters();
                //normalGyroDrive(0.25, 0, 0.3);
                //normalDrive(0.45, 40, 40);
                normalstrafeDrive(0.6, -58, 58);
                sleep(100);
                normalGyroDrive(0.2, 0, 0.3);
                Katyusha();
                Trigger(0.35);
                //sleep(100);
                Intake(-1);
                normalDrive(0.5, -40, -40);
                //normalDrive(0.5, 45, 45);
                //Intake(0);
                //normalGyroDrive(0.3, 0, 0.5);
                //InterContinentalBallisticMissleSingle(3);
                //normalDrive(0.6,-18,-18);
                sleep(250);
                normalDrive(0.5, -16, -16);
                sleep(500);
                normalDrive(0.6, 54, 54);
                Intake(0);

                Katyusha();
                Trigger(0.35);
                Intake(-1);
                normalDrive(0.5, -80, -80);
                sleep(500);
                normalDrive(0.65, 80, 80);
                Intake(0);
                normalGyroDrive(0.2, 0, 0.3);

                InterContinentalBallisticMissleSingle(3);
                normalDrive(1, 140, 140);
                wRelease(0.4);
                sleep(100);
                normalDrive(1, -110, -110);
                //normalDrive(0.7,-10,-10);
                //sleep(500);
        }
            else{
                stop = 0;
                wRelease(0);
                normalDrive(0.55, 156, 156);
                InterContinentalBallisticMissleRising();
                normalGyroDrive(0.25, 0, 0.3);
                normalDrive(0.45, 40, 40);
                normalstrafeDrive(0.5, -96, 96);
                wRelease(0.4);
                normalDrive(0.7, -102, -102);
                normalstrafeDrive(0.55, -55, 55);
                normalstrafeDrive(0.4, 15, -15);
                armAutonom(530);
                fingerAutonom(0);
                //sleep(100);
                normalDistanceDrive(0.25, 60, 23);
                sleep(200);
                fingerAutonom(1);
                sleep(250);
                armAutonom(150);
                //sleep(100);
                stop = 1;
                sleep(100);
                normalstrafeDrive(0.5, 33, -33);
                normalDrive(0.6, 117, -117);
                sleep(100);
                normalDrive(0.651, -70, -70);
                sleep(100);
                stop = 0;
                ///sleep(100);
                armAutonom(480);
                sleep(100);
                fingerAutonom(0);
                sleep(100);
                normalDrive(0.6, 5, 5);
                normalstrafeDrive(1, -60, 60);
                normalDrive(1, -20, -20);
                TeleThread.interrupt();
                stop();
        }

        if (0 == 1) {
            stop = 0;
            wRelease(0);
            normalDrive(0.55, 156, 156);
            InterContinentalBallisticMissleRising();
            normalGyroDrive(0.25, 0, 0.3);
            normalDrive(0.45, 40, 40);
            normalstrafeDrive(0.5, -96, 96);
            wRelease(0.4);
            normalDrive(0.7, -102, -102);
            normalstrafeDrive(0.55, -55, 55);
            normalstrafeDrive(0.4, 15, -15);
            armAutonom(530);
            fingerAutonom(0);
            //sleep(100);
            normalDistanceDrive(0.25, 60, 23);
            sleep(200);
            fingerAutonom(1);
            sleep(250);
            armAutonom(150);
            //sleep(100);
            stop = 1;
            sleep(100);
            normalstrafeDrive(0.5, 33, -33);
            normalDrive(0.6, 117, -117);
            sleep(100);
            normalDrive(0.651, -70, -70);
            sleep(100);
            stop = 0;
            ///sleep(100);
            armAutonom(480);
            sleep(100);
            fingerAutonom(0);
            sleep(100);
            normalDrive(0.6, 5, 5);
            normalstrafeDrive(1, -60, 60);
            normalDrive(1, -20, -20);
            TeleThread.interrupt();
        }
        if (1 != 1) {
            //wRelease(0);
            normalDrive(0.6, 159, 159);
            InterContinentalBallisticMissleShooters();
            //normalGyroDrive(0.25, 0, 0.3);
            //normalDrive(0.45, 40, 40);
            normalstrafeDrive(0.6, -58, 58);
            normalGyroDrive(0.25, 0, 0.4);
            Trigger(0.35);
            Intake(-1);
            normalDrive(0.6, -50, -50);
            normalDrive(0.5, 50, 50);
            Intake(0);
            normalGyroDrive(0.25, 0, 0.4);
            InterContinentalBallisticMissleSingle(3);

            normalDrive(0.7, 90, 90);
            normalstrafeDrive(0.7, 29, -29);
            wRelease(0.4);
            normalDrive(0.8, -150, -150);
            normalstrafeDrive(0.8, -120, 120);
            normalstrafeDrive(0.45, 15, -15);
            armAutonom(530);
            fingerAutonom(0);
            //sleep(100);
            normalDistanceDrive(0.3, 60, 23);
            sleep(200);
            fingerAutonom(1);
            //sleep(250);
            armAutonom(150);
            //sleep(100);
            stop = 1;
            sleep(100);
            normalstrafeDrive(0.7, 90, -90);
            normalDrive(0.7, 118, -118);
            sleep(100);
            normalDrive(0.7, -120, -120);
            stop = 0;
            ///sleep(100);
            armAutonom(480);
            sleep(100);
            fingerAutonom(0);
            sleep(100);
            normalDrive(1, 10, 10);

        }
        if (1 == 2){
        wRelease.setPosition(0);
        fingerAutonom(0.5);
        normalDrive(0.7, 155, 155);

        //InterContinentalBallisticMissleShooters();
        //normalGyroDrive(0.25, 0, 0.3);
        //normalDrive(0.45, 40, 40);
        normalstrafeDrive(0.6, -58, 58);
        //normalGyroDrive(0.3, 0, 0.5);
        Katyusha();
        Trigger(0.35);
        Intake(-1);
        normalDrive(0.6, -45, -45);
        normalDrive(0.5, 45, 45);
        Intake(0);
        //normalGyroDrive(0.3, 0, 0.5);
        InterContinentalBallisticMissleSingle(3);

        normalDrive(1, 43, 43);
        normalDrive(1, 55, -55);

        wRelease(0.4);
        normalstrafeDrive(1, 10, -10);
        normalDrive(1, -52, 52);
        normalDrive(1, -100, -100);
        normalstrafeDrive(1, -85, 85);
        normalstrafeDrive(0.4, 15, -15);
        armAutonom(530);
        fingerAutonom(0);
        //sleep(100);
        normalDistanceDrive(0.25, 60, 22.5);
        sleep(200);
        fingerAutonom(1);
        sleep(200);
        armAutonom(150);
        //sleep(100);
        stop = 1;
        sleep(100);
        normalstrafeDrive(1, 90, -90);
        normalDrive(1, 121, -121);
        sleep(100);
        normalDrive(1, -100, -100);
        stop = 0;
        ///sleep(100);
        armAutonom(320);
        sleep(100);
        fingerAutonom(0);
        sleep(100);
        normalDrive(1, 10, 10);
    }
if (4==1) {
    wRelease.setPosition(0);
    fingerAutonom(0.5);
    normalDrive(0.7, 155, 155);

    //InterContinentalBallisticMissleShooters();
    //normalGyroDrive(0.25, 0, 0.3);
    //normalDrive(0.45, 40, 40);
    normalstrafeDrive(0.6, -58, 58);
    sleep(100);
    normalGyroDrive(0.2, 0, 0.3);
    Katyusha();
    Trigger(0.35);
    //sleep(100);
    Intake(-1);
    normalDrive(0.5, -40, -40);
    //normalDrive(0.5, 45, 45);
    //Intake(0);
    //normalGyroDrive(0.3, 0, 0.5);
    //InterContinentalBallisticMissleSingle(3);
    //normalDrive(0.6,-18,-18);
    sleep(250);
    normalDrive(0.5, -16, -16);
    sleep(500);
    normalDrive(0.6, 54, 54);
    Intake(0);

    Katyusha();
    Trigger(0.35);
    Intake(-1);
    normalDrive(0.5, -80, -80);
    sleep(500);
    normalDrive(0.65, 80, 80);
    Intake(0);
    normalGyroDrive(0.2, 0, 0.3);

    InterContinentalBallisticMissleSingle(3);
    normalDrive(1, 140, 140);
    wRelease(0.4);
    sleep(100);
    normalDrive(1, -110, -110);
    //normalDrive(0.7,-10,-10);
    //sleep(500);
}
        /*if (4==2){

        wRelease.setPosition(0);
        fingerAutonom(0.5);
        normalDrive(1, 155, 155);

        //InterContinentalBallisticMissleShooters();
        //normalGyroDrive(0.25, 0, 0.3);
        //normalDrive(0.45, 40, 40);
        normalstrafeDrive(1, -58, 58);
        //sleep(100);
        normalGyroDrive(0.2, 0, 0.3);
        Katyusha();
        Trigger(0.35);

        //sleep(100);
        Intake(-1);
        normalDrive(1, -40, -40);
        //normalDrive(0.5, 45, 45);
        //Intake(0);
        //normalGyroDrive(0.3, 0, 0.5);
        //InterContinentalBallisticMissleSingle(3);
        //normalDrive(0.6,-18,-18);
        sleep(200);
        normalDrive(1, -16, -16);
        sleep(250);
        normalDrive(1, 54, 54);
        Intake(0);
normalGyroDrive(0.2,0,0.3);
        Katyusha();
        Trigger(0.35);
        normalDrive(1, 135, 135);
        normalstrafeDrive(1,-15,15);
        wRelease(0.4);
        //sleep(100);
        normalGyroDrive(0.2,0,0.3);

        Intake(-1);
        normalDrive(1, -190, -190);

        normalstrafeDrive(1, -90, 90);
        normalstrafeDrive(0.45, 15, -15);
        armAutonom(530);
        fingerAutonom(0);
        //sleep(100);
        normalDistanceDrive(0.27, 60, 22.5);
        sleep(200);
        fingerAutonom(1);
        sleep(200);
        armAutonom(150);
        }*/
        //sleep(100);
        /*stop = 1;
        sleep(100);
        normalstrafeDrive(1, 90, -90);
        normalDrive(1, 121, -121);
        sleep(100);
        normalDrive(1, -100, -100);
        stop = 0;
        ///sleep(100);
        armAutonom(320);
        sleep(100);
        fingerAutonom(0);
        sleep(100);
        normalDrive(1, 10, 10);

        */

        //normalDrive(0.7,-10,-10);
        /*normalDrive(1, 43, 43);
        normalDrive(1, 55, -55);

        wRelease(0.4);
        normalstrafeDrive(1, 10, -10);
        normalDrive(1, -52, 52);
        normalDrive(1, -100, -100);
        normalstrafeDrive(1, -85, 85);
        normalstrafeDrive(0.4, 15, -15);
        armAutonom(530);
        fingerAutonom(0);
        //sleep(100);
        normalDistanceDrive(0.25, 60, 22.5);
        sleep(200);
        fingerAutonom(1);
        sleep(200);
        armAutonom(150);
        //sleep(100);
        stop = 1;
        sleep(100);
        normalstrafeDrive(1, 90, -90);
        normalDrive(1, 121, -121);
        sleep(100);
        normalDrive(1, -100, -100);
        stop = 0;
        ///sleep(100);
        armAutonom(320);
        sleep(100);
        fingerAutonom(0);
        sleep(100);
        normalDrive(1, 10, 10);*/
    }

    public void Intake(double speed) {
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setPower(speed);
        sleep(500);
    }

    public void normalDistanceDrive(double speed, double distance, double buffer) {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// this creates the variables that will be calculated
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        distance2 = sensorDistance.getDistance(DistanceUnit.CM);
        if (distance2 < distance) {
            //this gets the absolute speed and converdets it into power for the motor.
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
        } else if (distance2 > distance) {
            //this gets the absolute speed and converdets it into power for the motor.
            FR.setPower(-Math.abs(speed));
            FL.setPower(-Math.abs(speed));
            BR.setPower(-Math.abs(speed));
            BL.setPower(-Math.abs(speed));
        }
        while (distance2 > distance + buffer || distance2 < distance - buffer && opModeIsActive()) {
            //inALoop="in DistanceDrive loop";
if (distance2>5000)
{break;}
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            distance2 = sensorDistance.getDistance(DistanceUnit.CM);
            if (distance2 < distance ) {
                //this gets the absolute speed and converdets it into power for the motor.
                FR.setPower(Math.abs(speed));
                FL.setPower(Math.abs(speed));
                BR.setPower(Math.abs(speed));
                BL.setPower(Math.abs(speed));
            } else if (distance2 > distance ) {
                //this gets the absolute speed and converdets it into power for the motor.
                FR.setPower(-Math.abs(speed));
                FL.setPower(-Math.abs(speed));
                BR.setPower(-Math.abs(speed));
                BL.setPower(-Math.abs(speed));
            }
        }
        //inALoop="Out of distance drive loop";777
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
    public void armAutonom(int pos) {
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*if (gamepad2.x) {
            // move to 0 degrees.
            Arm.setPower(0.2);

        } else if (gamepad2.y) {
            // move to 90 degrees.
            Arm.setPower(-0.2);
        } else {
            Arm.setPower(0);
        }
*/



            Arm.setPower(0.3);
            Arm.setTargetPosition(pos);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

while (Arm.isBusy()&&opModeIsActive()){
    telemetry.addData("pos",Arm.getCurrentPosition());
    telemetry.update();
        }
Arm.setPower(0);
Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void fingerAutonom(double pos) {


        finger.setPosition(pos);
        //telemetry.update();

        finger.setPosition(pos);

        for(int i =1;i<=1000;i++)
        {finger.setPosition(pos);}
        sleep(200);
    }

    public void wRelease(double pos) {
        wRelease.setPosition(pos);
        sleep(500);
    }

    public void Trigger(double pos) {


            Trigger.setPosition(pos);
        idle();
        sleep(500);
    }

    private void MSBAutonom() {
        double power = -0.94;
        ;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //sleep(300);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(400);
        Shooter.setPosition(0.3);
        sleep(400);
        InTake.setPower(-1);
        sleep(2500);
        InTake.setPower(0);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }

    private void BetterShooterAutonomV2(double power, long sleep) {
        //double power = -0.78;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //sleep(300);
        Launcher1.setPower(-power);
        Launcher2.setPower(-power);
        sleep(400);
        Shooter.setPosition(0.3);
        sleep(200);
        InTake.setPower(-1);
        sleep(sleep);
        InTake.setPower(0);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }

    private void SSBAutonom() {
        double power = -0.94;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //sleep(300);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(400);
        Shooter.setPosition(0.3);
        sleep(700);
        InTake.setPower(-1);
        sleep(1300);
        InTake.setPower(0);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }
    /*public void InterContinentalBallisticMissleRising() {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//normalGyroDrive(0.3,-2,0.6);
        normalDrive(0.3, -2.2, 2.2);
        Trigger.setPosition(0.0);
        Launcher1.setVelocity(-1890);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1890);
        sleep(1300);
        Shooter.setPosition(0.23);
        sleep(500);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,-6,0.6);
        normalDrive(0.3, -4.3, 4.3);
        sleep(500);
        Trigger.setPosition(0.0);
        Launcher1.setVelocity(-1910);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1910);
        //sleep(1300);
        Shooter.setPosition(0.55);
        sleep(500);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,3.2,0.35);
        normalDrive(0.3, 9.5, -9.5);
        sleep(500);
        Trigger.setPosition(0.0);
        Launcher1.setVelocity(-1898);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1898);
        //sleep(1300);
        Shooter.setPosition(0.87);
        sleep(1000);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);

            /*Trigger.setPosition(0.0);
            Launcher1.setVelocity(-2160);
            Launcher2.setVelocity(-2160);
            sleep(1550);
            Shooter.setPosition(0.1);
            sleep(250);
            Launcher1.setVelocity(-1990);
            Launcher2.setVelocity(-1990);
            sleep(500);
            Shooter.setPosition(0.5);
            sleep(250);
            Launcher1.setVelocity(-2100);
            Launcher2.setVelocity(-2100);
            sleep(500);
            Shooter.setPosition(0.88);
            sleep(1000);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);*/

    public void InterContinentalBallisticMissleRising() {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//normalGyroDrive(0.3,-2,0.6);
        //normalDrive(0.3, -2.2, 2.2);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1890);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1890);
        sleep(1300);
        Shooter.setPosition(0.23);
        sleep(250);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,-6,0.6);
        normalDrive(0.3, 4.3, -4.3);
        sleep(100);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1910);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1910);
        //sleep(1300);
        Shooter.setPosition(0.55);
        sleep(250);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,3.2,0.35);
        normalDrive(0.3, 7.5, -7.5);
        sleep(250);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1900);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1900);
        //sleep(1300);
        Shooter.setPosition(0.87);
        sleep(500);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);



    }
    public void InterContinentalBallisticMissleSingle(int pos) {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//normalGyroDrive(0.3,-2,0.6);
        if (pos == 1) {
            //normalDrive(0.3, -2.2, 2.2);
            Trigger.setPosition(0.87);
            Launcher1.setVelocity(-2200);//2250/13.72V      2220/13.62V
            Launcher2.setVelocity(-2050);
            sleep(1300);
            Shooter.setPosition(0.23);
            sleep(250);
            Shooter.setPosition(0);
            Launcher1.setVelocity(0);//2250/13.72V      2220/13.62V
            Launcher2.setVelocity(0);
        }
        if (pos == 2) {
            //normalGyroDrive(0.3,-6,0.6);
            //normalDrive(0.3, 4.3, -4.3);
            //sleep(100);
            Trigger.setPosition(0.87);
            Launcher1.setVelocity(-2050);//2250/13.72V      2220/13.62V
            Launcher2.setVelocity(-2050);
            sleep(1300);
            Shooter.setPosition(0.55);
            sleep(250);
            Shooter.setPosition(0);
            Launcher1.setVelocity(0);//2250/13.72V      2220/13.62V
            Launcher2.setVelocity(0);
        }
        //normalGyroDrive(0.3,3.2,0.35);
        //normalDrive(0.3, 7.5, -7.5);
        if (pos == 3) {
                //sleep(250);
            Trigger.setPosition(0.87);
            Launcher1.setVelocity(-2060);
            Launcher2.setVelocity(-2060);//1915
            sleep(1000);
            Shooter.setPosition(0.87);
            sleep(700);
            Shooter.setPosition(0);
            Launcher1.setVelocity(0);
            Launcher2.setVelocity(0);
        }


    }

    public void InterContinentalBallisticMissleShooters() {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//normalGyroDrive(0.3,-2,0.6);
        normalDrive(0.3, -2.2, 2.2);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1890);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1890);
        sleep(1000);
        Shooter.setPosition(0.23);
        sleep(200);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,-6,0.6);
        normalDrive(0.3, 4.3, -4.3);
        sleep(100);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1910);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1910);
        //sleep(1300);
        Shooter.setPosition(0.55);
        sleep(200);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,3.2,0.35);
        normalDrive(0.3, 7.5, -7.5);
        sleep(200);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1900);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1900);
        //sleep(1300);
        Shooter.setPosition(0.87);
        sleep(400);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);

            /*Trigger.setPosition(0.0);
            Launcher1.setVelocity(-2160);
            Launcher2.setVelocity(-2160);
            sleep(1550);
            Shooter.setPosition(0.1);
            sleep(250);
            Launcher1.setVelocity(-1990);
            Launcher2.setVelocity(-1990);
            sleep(500);
            Shooter.setPosition(0.5);
            sleep(250);
            Launcher1.setVelocity(-2100);
            Launcher2.setVelocity(-2100);
            sleep(500);
            Shooter.setPosition(0.88);
            sleep(1000);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);*/
            /*if (voltage >= 13.6 && voltage < 13.9) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2100);
                Launcher2.setVelocity(-2100);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-1930);
                Launcher2.setVelocity(-1930);
                sleep(750);
                Shooter.setPosition(0.77);
                sleep(250);
                Launcher1.setVelocity(-2100);
                Launcher2.setVelocity(-2100);
                sleep(750);
                Shooter.setPosition(0.82);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage >= 13.5 && voltage < 13.6) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2180);
                Launcher2.setVelocity(-2180);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2010);
                Launcher2.setVelocity(-2010);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2120);
                Launcher2.setVelocity(-2120);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage >= 13.4 && voltage < 13.5) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2185);
                Launcher2.setVelocity(-2185);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2032);
                Launcher2.setVelocity(-2035);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2138);
                Launcher2.setVelocity(-2138);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage < 13.4) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2185);
                Launcher2.setVelocity(-2185);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2032);
                Launcher2.setVelocity(-2035);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2138);
                Launcher2.setVelocity(-2138);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }*/
        /*Trigger.setPosition(0.0);
        Launcher1.setVelocity(-2200);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-2200);
        sleep(1300);
        Shooter.setPosition(0.23);
        sleep(500);
        Shooter.setPosition(0.6);
        sleep(600);
        Shooter.setPosition(0.84);
        sleep(600);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
*/

    }

    public void InterContinentalBallisticMissleFalling() {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//normalGyroDrive(0.3,-2,0.6);
        normalDrive(0.3, -2.2, 2.2);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-2155);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-2155);
        sleep(1300);
        Shooter.setPosition(0.23);
        sleep(500);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,-6,0.6);
        normalDrive(0.3, -3, 3);
        sleep(500);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-2155);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-2155);
        //sleep(1300);
        Shooter.setPosition(0.55);
        sleep(500);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,3.2,0.35);
        normalDrive(0.3, 7.9, -7.9);
        sleep(500);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-2155);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-2155);
        //sleep(1300);
        Shooter.setPosition(0.87);
        sleep(1000);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);

            /*Trigger.setPosition(0.0);
            Launcher1.setVelocity(-2160);
            Launcher2.setVelocity(-2160);
            sleep(1550);
            Shooter.setPosition(0.1);
            sleep(250);
            Launcher1.setVelocity(-1990);
            Launcher2.setVelocity(-1990);
            sleep(500);
            Shooter.setPosition(0.5);
            sleep(250);
            Launcher1.setVelocity(-2100);
            Launcher2.setVelocity(-2100);
            sleep(500);
            Shooter.setPosition(0.88);
            sleep(1000);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);*/
            /*if (voltage >= 13.6 && voltage < 13.9) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2100);
                Launcher2.setVelocity(-2100);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-1930);
                Launcher2.setVelocity(-1930);
                sleep(750);
                Shooter.setPosition(0.77);
                sleep(250);
                Launcher1.setVelocity(-2100);
                Launcher2.setVelocity(-2100);
                sleep(750);
                Shooter.setPosition(0.82);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage >= 13.5 && voltage < 13.6) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2180);
                Launcher2.setVelocity(-2180);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2010);
                Launcher2.setVelocity(-2010);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2120);
                Launcher2.setVelocity(-2120);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage >= 13.4 && voltage < 13.5) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2185);
                Launcher2.setVelocity(-2185);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2032);
                Launcher2.setVelocity(-2035);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2138);
                Launcher2.setVelocity(-2138);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage < 13.4) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2185);
                Launcher2.setVelocity(-2185);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2032);
                Launcher2.setVelocity(-2035);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2138);
                Launcher2.setVelocity(-2138);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }*/
        /*Trigger.setPosition(0.0);
        Launcher1.setVelocity(-2200);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-2200);
        sleep(1300);
        Shooter.setPosition(0.23);
        sleep(500);
        Shooter.setPosition(0.6);
        sleep(600);
        Shooter.setPosition(0.84);
        sleep(600);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
*/

    }

    public void Katyusha() {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double ICBM = -0.76;


        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-2130);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-2130);
        sleep(1000);
        Shooter.setPosition(0.23);
        sleep(500);
        Shooter.setPosition(0.6);
        sleep(500);
        Shooter.setPosition(0.86);
        sleep(500);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);

    }

  /*  public void encoderDrive(double speed, double leftInches, double rightInches) {

        double encPower = 0.2;
// this creates the variables that will be calculated

        double OldXL = leftEncoder.getCurrentPosition();
        double OldXR = rightEncoder.getCurrentPosition();

        double PosXL = leftEncoder.getCurrentPosition();
        double PosXR = rightEncoder.getCurrentPosition();


        double semn;

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;


        double WantedY = 0;
        double WantedXL = leftInches * TICKS_PER_CM_ENC;
        double WantedXR = rightInches * TICKS_PER_CM_ENC;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
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
        while (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy() && opModeIsActive()) {
            telemetry.addData("leftEN", leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN", rightEncoder.getCurrentPosition());
            telemetry.addData("frontEn", frontEncoder.getCurrentPosition());
            telemetry.update();
        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// resets all the data for the encoders.

        /*int XLSign = 1;
        int XRSign = 1;
        int LegitY = 0;
        double LegitXL = PosXL+WantedXL-leftEncoder.getCurrentPosition();
        double LegitXR = PosXR+WantedXR-rightEncoder.getCurrentPosition();
        boolean CloseEnough4Me=false;
        while(CloseEnough4Me==false)
        {
            LegitXL = PosXL+WantedXL-leftEncoder.getCurrentPosition();
            LegitXR = PosXR+WantedXR-rightEncoder.getCurrentPosition();

            telemetry.addData("Pos XL",PosXL);
            telemetry.addData("Wanted XL",WantedXL);
            telemetry.addData("Curr XL",leftEncoder.getCurrentPosition());
            telemetry.addData("Legit XL",LegitXL);
            telemetry.addData("Pos XR",PosXR);
            telemetry.addData("Wanted XR",WantedXR);
            telemetry.addData("Curr XR",rightEncoder.getCurrentPosition());
            telemetry.addData("Legit XR",LegitXR);
            telemetry.update();


            if (Math.abs(LegitXL)<CloseEnough&&Math.abs(LegitXR)<CloseEnough){
                CloseEnough4Me = true;
                break;
            }


            if (LegitXL<0) XLSign = -1;
            else XLSign = 1;
            if (Math.abs(LegitXL)>100)
            {
                FL.setPower(Math.abs(encSpeed)*XLSign);
                BL.setPower(Math.abs(encSpeed)*XLSign);
                telemetry.addData("Pos XL",PosXL);
                telemetry.addData("Wanted XL",WantedXL);
                telemetry.addData("Curr XL",leftEncoder.getCurrentPosition());
                telemetry.addData("Legit XL",LegitXL);
                telemetry.addData("Pos XR",PosXR);
                telemetry.addData("Wanted XR",WantedXR);
                telemetry.addData("Curr XR",rightEncoder.getCurrentPosition());
                telemetry.addData("Legit XR",LegitXR);
                telemetry.update();
            }

            if (LegitXR<0) XRSign = -1;
            else XRSign = 1;
            if (Math.abs(LegitXR)>100)
            {
                FR.setPower(Math.abs(encSpeed)*XRSign);
                BR.setPower(Math.abs(encSpeed)*XRSign);
                telemetry.addData("Pos XL",PosXL);
                telemetry.addData("Wanted XL",WantedXL);
                telemetry.addData("Curr XL",leftEncoder.getCurrentPosition());
                telemetry.addData("Legit XL",LegitXL);
                telemetry.addData("Pos XR",PosXR);
                telemetry.addData("Wanted XR",WantedXR);
                telemetry.addData("Curr XR",rightEncoder.getCurrentPosition());
                telemetry.addData("Legit XR",LegitXR);
                telemetry.update();
            }

        }



            x = (PosXL - OldXL) - (PosXR - OldXR);

            semn = x / Math.abs(x);

            FL.setPower(-semn * encPower);
            BL.setPower(-semn * encPower);
            if (Math.abs(x) < 50) break;

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

    public void strafeDrive(double speed, double leftInches, double rightInches) {

        double encPower = 0.2;
// this creates the variables that will be calculated

        double OldXL = leftEncoder.getCurrentPosition();
        double OldXR = rightEncoder.getCurrentPosition();

        double PosXL = leftEncoder.getCurrentPosition();
        double PosXR = rightEncoder.getCurrentPosition();

        double x = (PosXL - OldXL) - (PosXR - OldXR);

        double semn;
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BL.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
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


        while (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy() && opModeIsActive()) {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:", FL.getCurrentPosition());
            telemetry.update();
        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PosXL = leftEncoder.getCurrentPosition();
        PosXR = rightEncoder.getCurrentPosition();
// resets all the data for the encoders.
        x = (PosXL - OldXL) - (PosXR - OldXR);

        while (Math.abs(x) > 50 && opModeIsActive()) {
            PosXL = leftEncoder.getCurrentPosition();
            PosXR = rightEncoder.getCurrentPosition();
            telemetry.addData("leftEN", leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN", rightEncoder.getCurrentPosition());
            telemetry.addData("frontEn", frontEncoder.getCurrentPosition());
            telemetry.update();


            x = (PosXL - OldXL) - (PosXR - OldXR);

            semn = x / Math.abs(x);

            FL.setPower(-semn * encPower);
            BL.setPower(-semn * encPower);
            if (Math.abs(x) < 50) break;

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
*/
    public void normalDrive(double speed, double leftInches, double rightInches) {

// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
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
        while (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy() && opModeIsActive()) {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:", FL.getCurrentPosition());
            telemetry.update();
            //initDiff = frontEncoder.getCurrentPosition() - leftEncoder.getCurrentPosition();
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

    public void normalstrafeDrive(double speed, double leftInches, double rightInches) {
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BL.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
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


        while (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy() && opModeIsActive()) {
            telemetry.addData("Target:", newLeftFrontTarget);
            telemetry.addData("Pos:", FL.getCurrentPosition());
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

    public void normalGyroDrive(double speed, double angle, double buffer) {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// this creates the variables that will be calculated
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.firstAngle < angle) {
            //this gets the absolute speed and converdets it into power for the motor.
            FR.setPower(-Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(-Math.abs(speed));
            BL.setPower(Math.abs(speed));
        } else if (angles.firstAngle > angle) {
            //this gets the absolute speed and converdets it into power for the motor.
            FR.setPower(Math.abs(speed));
            FL.setPower(-Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(-Math.abs(speed));
        }
        while (angles.firstAngle > angle + buffer || angles.firstAngle < angle - buffer && opModeIsActive()) {
            //    inALoop = "in normalGyro loop";
            if (gamepad2.back) {
                break;
            }
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (angles.firstAngle < angle) {
                //this gets the absolute speed and converdets it into power for the motor.
                FR.setPower(-Math.abs(speed));
                FL.setPower(Math.abs(speed));
                BR.setPower(-Math.abs(speed));
                BL.setPower(Math.abs(speed));
            } else if (angles.firstAngle > angle) {
                //this gets the absolute speed and converdets it into power for the motor.
                FR.setPower(Math.abs(speed));
                FL.setPower(-Math.abs(speed));
                BR.setPower(Math.abs(speed));
                BL.setPower(-Math.abs(speed));
            }
        }
        //inALoop = "out of normal gyro loop";

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

    public void SingleShotAutonom() {
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1700);
        Shooter.setPosition(0.3);
        sleep(500);
        Shooter.setPosition(0);
        sleep(100);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }

    /*public void MultiShotAutonom(){
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1800);
        Shooter.setPosition(0.3);
        sleep(600);
        Shooter.setPosition(0);
        sleep(100);
        Shooter.setPosition(0.3);
        sleep(600);
        Shooter.setPosition(0);
        sleep(250);
        Shooter.setPosition(0.3);
        sleep(600);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);



    }*/
    public void MultiShottestAutonom() {
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1700);
        Shooter.setPosition(0.3);
        sleep(100);
        Shooter.setPosition(0);
        sleep(300);
        Shooter.setPosition(0.3);
        sleep(100);
        Shooter.setPosition(0);
        sleep(300);
        Shooter.setPosition(0.3);
        sleep(300);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);


    }

    public void IntakeAutonom(double power) {
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
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

        while (mag_crm.getState()) {
            cremaliera_Servo.setPower(viteza);
            if (!mag_crm.getState()) {
                cremaliera_Servo.setPower(0);
                break;
            }
        }
        if (!mag_crm.getState()) {
            cremaliera_Servo.setPower(0);
        }


    }

    public void GlisieraAutonom(long sleep, double speed) {

        DcMotor glisiera = hardwareMap.get(DcMotorEx.class, "GLS");
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DigitalChannel glsMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");


        glisiera.setPower(speed);
        sleep(sleep);
        glisiera.setPower(0);
    }


    public void move(double x, double y, double rot, double cmdistance) {
        //do some sort of distance conversion
        double distance;
        distance = cmdistance * TICKS_PER_CM;
        if (FL.getCurrentPosition() < FL.getCurrentPosition() + distance && FR.getCurrentPosition() < FR.getCurrentPosition() + distance
                && BR.getCurrentPosition() < BR.getCurrentPosition() + distance && BL.getCurrentPosition() < BL.getCurrentPosition() + distance) {
            //drive at a power
            setPower(x, y, rot);
        } else {
            setPower(0.0, 0.0, 0.0);
        }
        while (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy() && opModeIsActive()) {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:", FL.getCurrentPosition());
            telemetry.update();
        }
        setPower(0.0, 0.0, 0.0);
    }

    public void setPower(double x, double y, double rot) {
        double frontLeftMotorPower = y - x - rot;
        double frontRightMotorPower = y + x + rot;
        double backLeftMotorPower = y + x - rot;
        double backRightMotorPower = y - x + rot;

        double motorPowers[] = {Math.abs(frontLeftMotorPower),
                Math.abs(backRightMotorPower),
                Math.abs(backLeftMotorPower),
                Math.abs(frontRightMotorPower)};

        Arrays.sort(motorPowers);

        if (motorPowers[3] != 0) {
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
                //average1 = avg1;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                ringCount=1;
                //average1 = avg1;
            }else{
                position = RingPosition.NONE;
                ringCount=0;
                //average1 = avg1;
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

        for (int i = 1; i <= 2000; i++) {
            wRelease.setPosition(position);
        }

        telemetry.addData("Servo Position", wRelease.getPosition());
        telemetry.update();
    }

    public class DriveThread extends Thread {
        public DriveThread() {
            this.setName("DriveThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {


            while (!isInterrupted()&&opModeIsActive()) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("distance",distance2);
                telemetry.addData("Trigger pos",Trigger.getPosition());
                telemetry.addData("Shooter pos",Shooter.getPosition());
                telemetry.update();
if (stop ==1)
{
    Arm.setTargetPosition(150);
    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Arm.setPower(0.3);
}

            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.


            //Logging.log("end of thread %s", this.getName());
        }
    }

}