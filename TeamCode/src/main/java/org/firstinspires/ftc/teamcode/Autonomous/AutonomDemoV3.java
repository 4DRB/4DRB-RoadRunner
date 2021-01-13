package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous(name = "TestAutonomousV3")
public class AutonomDemoV3 extends LinearOpMode {

    //hardware variables
    OpenCvCamera phoneCam = null;
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    //Intake intake = new Intake(hardwareMap);
    //static and other variables
    static double ringCount = 0;
    int rect1X = 70;
    int rect1Y = 170;
    int rect2X =70;
    int rect2Y = 170;

    final double TICKS_PER_REV = 537.6 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference 9.6
    double TICKS_PER_CM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    int initDiff,lastDiff,diffDiff;
    Encoder leftEncoder, rightEncoder, frontEncoder;


    @Override
    public void runOpMode()
    {
//hardware maps
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //detecting the starter stack height.
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));
        //tells the phone which detector to use
        phoneCam.setPipeline(new RingDetectingPipeline());
        //starts the stream
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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




        waitForStart();




        if(ringCount == 1){
            encoderDrive(0.5,145,145);
            strafeDrive(0.3,48,-48);
            MultiShotAutonom();
            IntakeAutonom(1);
            encoderDrive(0.3,30,-30);
            sleep(3000);
            IntakeAutonom(0);
        }else if(ringCount == 4){
            encoderDrive(0.5,145,145);
            strafeDrive(0.3,47,-47);
            MultiShotAutonom();
            IntakeAutonom(1);
            encoderDrive(0.3,-30,-30);
            sleep(1500);
            encoderDrive(0.3,-11,-11);
            sleep(1500);
            encoderDrive(0.3,-11,-11);
            sleep(2500);
            IntakeAutonom(0);
            encoderDrive(0.5,52,52);
            MultiShotAutonom();
            encoderDrive(0.5,140,143);
            strafeDrive(0.4,-15,15);

            encoderDrive(0.5,30,-30);
            encoderDrive(0.5,-30,30);
            encoderDrive(0.5,-100,-103);



            //drive somewhere else
        }else{
            //drive to a different spot
            encoderDrive(0.5,145,145);
            strafeDrive(0.3,48,-48);
            MultiShotAutonom();
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
        double power = -1;
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


    class RingDetectingPipeline extends OpenCvPipeline{

        Mat YCbCr = new Mat();
        Mat outPut = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            //image conversion
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //copying input to output
            input.copyTo(outPut);

            //creating the top rectangle
            Rect rect1 = new Rect(rect1X, rect1Y, 40, 30);


            //creating the bottom rectangle
            Rect rect2 = new Rect(rect2X, rect2Y, 40, 30);

            Scalar rectangleColor = new Scalar(0,0,255);

            //drawing rectangles on screen
            Imgproc.rectangle(outPut, rect1, rectangleColor,2);

            Imgproc.rectangle(outPut, rect2, rectangleColor,2);


            //cropping the image for stack height

            //cropping YCbCr, putting it on lowerCrop mat
            lowerCrop = YCbCr.submat(rect1);
            //cropping YCbCr, putting it on upperCrop mat
            upperCrop = YCbCr.submat(rect2);

            //taking the orange color out, placing on mat
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            //taking the orange color out, placing on mat
            Core.extractChannel(upperCrop, upperCrop, 2);

            //take the raw average data, put it on a Scalar variable
            Scalar lowerAverageOrange = Core.mean(lowerCrop);

            //take the raw average data, put it on a Scalar variable
            Scalar upperAverageOrange = Core.mean(lowerCrop);

            //finally, taking the first value of the average and putting it in a variable
            double finalLowerAverage = lowerAverageOrange.val[0];

            double finalUpperAverage = upperAverageOrange.val[0];

            //comparing average values
            if(finalLowerAverage > 15 && finalLowerAverage < 130 && finalUpperAverage < 130){
                ringCount = 4.0;
            }else if(finalLowerAverage > 10 && finalUpperAverage < 15 && finalLowerAverage > 10 && finalUpperAverage < 15){
                ringCount = 0.0;
            }else{
                ringCount = 1.0;
            }
            //this will be what we are showing on the viewport
            return outPut;
        }
    }
}
