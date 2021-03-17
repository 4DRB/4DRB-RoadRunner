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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OdometryAutonomous.OdometryGlobalCoordinatePosition;
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
public class AutonomDemoV8 extends LinearOpMode
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

    final double TICKS_PER_REV_ENC = 8192 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION_ENC = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM_ENC = 6;     // For figuring circumference 9.6
    double TICKS_PER_CM_ENC = 434;//(TICKS_PER_REV_ENC * DRIVE_GEAR_REDUCTION_ENC) / (WHEEL_DIAMETER_CM_ENC * 3.1415);

    int initDiff,lastDiff,diffDiff;
    Encoder leftEncoder, rightEncoder, frontEncoder;
    OpenCvCamera webCam = null;
    OdometryGlobalCoordinatePosition globalPositionUpdate;




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

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

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
        telemetry.addData("Suntem in cazul",ringCount);
        telemetry.addData("Average",pipeline.avg1);
        telemetry.update();


        waitForStart();


        telemetry.addData("Suntem in cazul",ringCount);
        telemetry.addData("Average",pipeline.getAnalysis());
        telemetry.update();

        if(ringCount == 1.0){

            Shooter.setPosition(0.1);
            Shooter.setPosition(0);
            wRelease.setPosition(0);
            normalDrive(0.6,145,145);
            normalstrafeDrive(0.6,51,-51);
            normalDrive(0.5,5,-5);
            IntakeAutonom(0);
            BetterShooterAutonomV2(0.935,2250);
            sleep(100);//normalstrafeDrive(0.4,-7,7);
            normalDrive(0.5,-5,5);
            //arunca gogosile,se duca sa mai ia gogosi
            IntakeAutonom(-1);
            normalDrive(0.5,-34,-34);
            sleep(1000);
            normalDrive(0.6,34,34);
            normalDrive(0.5,5,-5);
            IntakeAutonom(0);
            BetterShooterAutonomV2(0.935,850);
            sleep(100);//normalstrafeDrive(0.4,-7,7);
            normalDrive(0.5,-5,5);

            //arunca gogosile,se duce la  patrat 2

            normalDrive(0.6,101,101);
            normalstrafeDrive(0.5,30,-30);

            wRelease.setPosition(0.4);

            for(int i =1;i<=2000;i++)
            {wRelease.setPosition(0.4);}
            sleep(500);

            normalDrive(0.6,-70,-70);

            normalDrive(0.6,119,-119);
            sleep(100);
            normalstrafeDrive(0.5,15,-15);
            normalDrive(0.5,104,104);


            CremalieraAutonom(-0.8);
            ClampAutonom(0);
            sleep(500);
            GlisieraAutonom(200,1);

            normalDrive(0.6,-120,-120);
            normalDrive(0.6,-115,115);
            ClampAutonom(0.3);
            CremalieraAutonom(0.9);
            //drive somewhere else
            sleep(1000);
            stop();
        } else if(ringCount == 4.0){
            Shooter.setPosition(0.1);
            Shooter.setPosition(0);
            wRelease.setPosition(0);
            normalDrive(0.65,150,150);
            normalstrafeDrive(0.4,55,-55);
            MSBAutonom();
            normalstrafeDrive(0.4,-5,5);
            //arunca gogosile,se duca sa mai ia gogosi
            IntakeAutonom(-1);
            normalDrive(0.4,-27,-27);
            sleep(900);
            normalDrive(0.4,-10,-10);
            sleep(1100);
            normalDrive(0.4,-13,-13);
            sleep(1300);
            normalDrive(0.7,34,38);
            IntakeAutonom(0);
            //strafeDrive(0.4,-3,3);
            //normalstrafeDrive(0.4,7,-7);
            normalDrive(0.5,5,-5);
            MSBAutonom();
            //normalstrafeDrive(0.4,-7,7);
            normalDrive(0.5,-5,5);

            IntakeAutonom(-1);
            normalDrive(0.5,-65,-65);
            sleep(800);
            normalDrive(0.8,67,67);
            normalDrive(0.5,4,-4);
            IntakeAutonom(0);
            SSBAutonom();
            //normalstrafeDrive(0.4,-7,7);
            normalDrive(0.5,-4,4);
            //arunca gogosile,se duce la  patrat 3
            normalDrive(1,130,130);
            normalstrafeDrive(0.7,-25,25);

            normalDrive(1,30,-30);
            wRelease.setPosition(0.4);

            for(int i =1;i<=2000;i++)
            {wRelease.setPosition(0.4);}
            sleep(500);
            normalDrive(0.75,-30,30);
            normalDrive(1,-85,-85);


        }else{
            //drive to a different spot
            Shooter.setPosition(0.1);
            Shooter.setPosition(0);
            wRelease.setPosition(0);
            sleep(100);
            normalDrive(0.5,188,188);
            wRelease.setPosition(0.4);
            normalstrafeDrive(0.5,63,-63);

            normalDrive(0.5,-40,-40);

            //MultiShottestAutonom();
            MSBAutonom();
            //normalstrafeDrive(0.5,-5,5);
            sleep(100);
            normalDrive(0.5,118,-118);
            normalDrive(0.5,75,75);

            CremalieraAutonom(-0.8);
            ClampAutonom(0);
            sleep(500);
            GlisieraAutonom(200,1);



            normalDrive(0.5,-135,-135);
            normalDrive(0.5,58.6,-58.6);
            normalDrive(0.5,20,20);

            ClampAutonom(0.3);
            CremalieraAutonom(0.8);

            normalstrafeDrive(0.5,-10,10);
            normalDrive(0.6,-30,-30);
            normalDrive(0.5,-58.6,58.6);
            stop();
        }

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

    public void encoderDrive(double speed, double leftInches, double rightInches){

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
*/
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
    }
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