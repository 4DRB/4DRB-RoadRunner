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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
public class DoYouLikeMyCar extends LinearOpMode
{
    static double ringCount = 0;

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

    final double TICKS_PER_REV_ENC = 8192 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION_ENC = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM_ENC = 6;     // For figuring circumference 9.6
    double TICKS_PER_CM_ENC = 434;//(TICKS_PER_REV_ENC * DRIVE_GEAR_REDUCTION_ENC) / (WHEEL_DIAMETER_CM_ENC * 3.1415);

    int initDiff,lastDiff,diffDiff;
    Encoder leftEncoder, rightEncoder, frontEncoder;
    OpenCvCamera webCam = null;
    BNO055IMU imu;
    private Orientation angles;
    public DistanceSensor sensorRange;
    Rev2mDistanceSensor sensorDistance;




    @Override
    public void runOpMode()
    {
// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);




        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");


        FL = hardwareMap.get(DcMotor.class, "leftFront");
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
        sleep(1000);

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorDistance= (Rev2mDistanceSensor)sensorRange;

        waitForStart();



       /* while (opModeIsActive())
        {
            if (gamepad1.a)
            {


                while (sensorDistance.getDistance(DistanceUnit.CM)>68||sensorDistance.getDistance(DistanceUnit.CM)<63)
                {
                    if (sensorDistance.getDistance(DistanceUnit.CM)>68)normalstrafeDrive(0.5,5,-5);
                    else if (sensorDistance.getDistance(DistanceUnit.CM)<63)normalstrafeDrive(0.5,-5,-5);
                    telemetry.addData("we in AutoPos",1);
                    telemetry.addData("distance",sensorDistance.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                //m_robotDrive.stop();

                while (angles.firstAngle>2.5||angles.firstAngle<-2.5){
                    if (angles.firstAngle>2.5)normalDrive(0.5,2,-2);
                    else if (angles.firstAngle<-2.5)normalDrive(0.5,2,-2);
                    telemetry.addData("we in AutoPos",2);
                    telemetry.addData("angle",angles.firstAngle);
                    telemetry.update();
                }

                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
            }

            if (gamepad1.b)
            {
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled      = true;
                parameters.loggingTag          = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                imu.initialize(parameters);

                telemetry.addData("we in SetIMU",1);
                telemetry.update();
            }



    }*/

        while (opModeIsActive())
        {normalGyroDrive(0.5,0);}

    }

    private void MSBAutonom() {
        double power = -0.94;;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
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
    private void BetterShooterAutonomV2(double power,long sleep) {
        //double power = -0.78;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
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
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
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

   /* public void encoderDrive(double speed, double leftInches, double rightInches){

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
        double WantedXL = leftInches*TICKS_PER_CM_ENC;
        double WantedXR = rightInches*TICKS_PER_CM_ENC;

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
            telemetry.addData("leftEN",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN",rightEncoder.getCurrentPosition());
            telemetry.addData("frontEn",frontEncoder.getCurrentPosition());
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

        PosXL = leftEncoder.getCurrentPosition();
        PosXR = rightEncoder.getCurrentPosition();

        double x=(PosXL-OldXL)-(PosXR-OldXR);

        while (Math.abs(x)>50&&opModeIsActive()){
            PosXL = leftEncoder.getCurrentPosition();
            PosXR = rightEncoder.getCurrentPosition();
            telemetry.addData("leftEN",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN",rightEncoder.getCurrentPosition());
            telemetry.addData("SE CORECTEAZA POG!1!!",1);
            telemetry.update();


            x=(PosXL-OldXL)-(PosXR-OldXR);

            semn = x / Math.abs(x);

            FL.setPower(-semn*encPower);
            BL.setPower(-semn*encPower);
            if (Math.abs(x)<50)break;

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
    public void strafeDrive(double speed, double leftInches, double rightInches){

        double encPower = 0.2;
// this creates the variables that will be calculated

        double OldXL = leftEncoder.getCurrentPosition();
        double OldXR = rightEncoder.getCurrentPosition();

        double PosXL = leftEncoder.getCurrentPosition();
        double PosXR = rightEncoder.getCurrentPosition();

        double x=(PosXL-OldXL)-(PosXR-OldXR);

        double semn;
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
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PosXL = leftEncoder.getCurrentPosition();
        PosXR = rightEncoder.getCurrentPosition();
// resets all the data for the encoders.
        x=(PosXL-OldXL)-(PosXR-OldXR);

        while (Math.abs(x)>50&&opModeIsActive()){
            PosXL = leftEncoder.getCurrentPosition();
            PosXR = rightEncoder.getCurrentPosition();
            telemetry.addData("leftEN",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN",rightEncoder.getCurrentPosition());
            telemetry.addData("frontEn",frontEncoder.getCurrentPosition());
            telemetry.update();


            x=(PosXL-OldXL)-(PosXR-OldXR);

            semn = x / Math.abs(x);

            FL.setPower(-semn*encPower);
            BL.setPower(-semn*encPower);
            if (Math.abs(x)<50)break;

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
    }*/
    public void normalDrive(double speed, double leftInches, double rightInches){

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
    public void normalGyroDrive(double speed,double angle){
FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// this creates the variables that will be calculated
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.firstAngle<angle){
        //this gets the absolute speed and converdets it into power for the motor.
        FR.setPower(-Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(-Math.abs(speed));
        BL.setPower(Math.abs(speed));}
        else if (angles.firstAngle>angle){
            //this gets the absolute speed and converdets it into power for the motor.
            FR.setPower(Math.abs(speed));
            FL.setPower(-Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(-Math.abs(speed));}
        while(angles.firstAngle>angle+10||angles.firstAngle<angle-6 && opModeIsActive())
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (angles.firstAngle<angle-2){
                //this gets the absolute speed and converdets it into power for the motor.
                FR.setPower(-Math.abs(speed));
                FL.setPower(Math.abs(speed));
                BR.setPower(-Math.abs(speed));
                BL.setPower(Math.abs(speed));}
            else if (angles.firstAngle>angle+2){
                //this gets the absolute speed and converdets it into power for the motor.
                FR.setPower(Math.abs(speed));
                FL.setPower(-Math.abs(speed));
                BR.setPower(Math.abs(speed));
                BL.setPower(-Math.abs(speed));}
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
    public void normalstrafeDrive(double speed, double leftInches, double rightInches){
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
    public void MultiShottestAutonom(){
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
    public void GlisieraAutonom(long sleep,double speed) {

        DcMotor glisiera = hardwareMap.get(DcMotorEx.class, "GLS");
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DigitalChannel glsMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");


        glisiera.setPower(speed);
        sleep(sleep);
        glisiera.setPower(0);
    }


    /*public void move(double x, double y, double rot, double cmdistance){
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




    public void WRealeaseLeftAutonom(double position) {
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");

        wRelease.setPosition(position);

        for(int i =1;i<=2000;i++)
        {wRelease.setPosition(position);}

        telemetry.addData("Servo Position", wRelease.getPosition());
        telemetry.update();
    }



}
