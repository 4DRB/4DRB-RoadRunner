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
public class DemoAccDecc extends LinearOpMode
{
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
    public void runOpMode()
    {
// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");



        waitForStart();
        WRealeaseLeftAutonom(0.0);

        FwAccDecc(120000,0.5);
        Stop();
        WRealeaseLeftAutonom(0.4);

        EncoderLeft(17000,0.3);
        Stop();
        Forward(300,0.5);
        Stop();
        //FwAccDecc(90000,-0.5);


    }


    public int getPosition(){
        return intPosition;
    }
    public void FwAccDecc(int target , double power)
    {   //init section

        double firstDistance = target * 0.1;
        double secondDistance = target * 0.9;
        double lastDistance = target;

        double firstPower = power * 0.4;
        double secondPower = power * 0.8;
        double lastPower = power * 0.2;

        double currDistance = leftEncoder.getCurrentPosition();

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

        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ///////////////////////////////////////////////////code section

        while (currDistance < lastDistance)
        {
            currDistance = leftEncoder.getCurrentPosition();
            if (currDistance<firstDistance)
            {
                TrightDrive.setPower(firstPower);
                BrightDrive.setPower(firstPower);
                TleftDrive.setPower(firstPower);
                BleftDrive.setPower(firstPower);
                telemetry.addLine("We at firstDistance");
            }
            if (currDistance<secondDistance&&currDistance>=firstDistance)
            {
                TrightDrive.setPower(secondPower);
                BrightDrive.setPower(secondPower);
                TleftDrive.setPower(secondPower);
                BleftDrive.setPower(secondPower);
                telemetry.addLine("We at secondDistance");
            }
            if (currDistance<lastDistance&&currDistance>=secondDistance)
            {
                TrightDrive.setPower(lastPower);
                BrightDrive.setPower(lastPower);
                TleftDrive.setPower(lastPower);
                BleftDrive.setPower(lastPower);
                telemetry.addLine("We at lastDistance");
            }
            telemetry.addData("currDistance" , currDistance);
            telemetry.update();
        }
        TrightDrive.setPower(0);
        BrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BleftDrive.setPower(0);

        TleftDrive.close();
        TrightDrive.close();
        BleftDrive.close();
        BrightDrive.close();
        /**TrightDrive.setPower(-power + power*-offTRD/100);
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
        BrightDrive.close();**/
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

        TleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        TleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        TleftDrive.setTargetPosition(-target);
        BleftDrive.setTargetPosition(-target);
        TrightDrive.setTargetPosition(-target);
        BrightDrive.setTargetPosition(-target);

        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TleftDrive.setPower(power);
        BleftDrive.setPower(power);
        TrightDrive.setPower(power);
        BrightDrive.setPower(power);
        while(TleftDrive.isBusy() && BleftDrive.isBusy() && opModeIsActive()&& TrightDrive.isBusy() && opModeIsActive()&& BrightDrive.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",TleftDrive.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Status:", "At pos");
        telemetry.update();
        /* We've reached position once that loop above this ends. Stop motors from moving. */
        TleftDrive.setPower(0);
        BleftDrive.setPower(0);
        TrightDrive.setPower(0);
        BrightDrive.setPower(0);

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
    public void EncoderRight(int target  , double power) {
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

        int currPosition = frontEncoder.getCurrentPosition();

        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ////////////////////////////////////////////////////////////////////code side

        while (currPosition<target)
        {
            TleftDrive.setPower(-power);
            BleftDrive.setPower(power);
            TrightDrive.setPower(power);
            BrightDrive.setPower(-power);
        }
        TrightDrive.setPower(0);
        BrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BleftDrive.setPower(0);

        TleftDrive.close();
        TrightDrive.close();
        BleftDrive.close();
        BrightDrive.close();
    }
    public void EncoderLeft(int target  , double power) {
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

        int currPosition = frontEncoder.getCurrentPosition();

        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ////////////////////////////////////////////////////////////////////code side

        while (currPosition>-target)
        {
            currPosition=frontEncoder.getCurrentPosition();
            TleftDrive.setPower(-power);
            BleftDrive.setPower(power);
            TrightDrive.setPower(power);
            BrightDrive.setPower(-power);
            telemetry.addData("We are at",currPosition);
            telemetry.update();
        }
        TrightDrive.setPower(0);
        BrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BleftDrive.setPower(0);

        TleftDrive.close();
        TrightDrive.close();
        BleftDrive.close();
        BrightDrive.close();
    }
    public void EncoderBack(int target  , double power) {
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

        int currPosition = frontEncoder.getCurrentPosition();

        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ////////////////////////////////////////////////////////////////////code side

        while (currPosition>target)
        {
            currPosition=frontEncoder.getCurrentPosition();
            TleftDrive.setPower(-power);
            BleftDrive.setPower(-power);
            TrightDrive.setPower(-power);
            BrightDrive.setPower(-power);
            telemetry.addData("We are at",currPosition);
            telemetry.update();
        }
        TrightDrive.setPower(0);
        BrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BleftDrive.setPower(0);

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
    public void WRealeaseLeftAutonom(double position) {
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");

        wRelease.setPosition(position);
        for(int i =1;i<=2000;i++)
        {wRelease.setPosition(position);}

        telemetry.addData("Servo Position", wRelease.getPosition());
        telemetry.update();
    }

}