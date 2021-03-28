package org.firstinspires.ftc.teamcode.TeleOp;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "ParaParaDancin")
public class ParaParaDancin extends OpMode {

    private Boolean CrSensLastDep = true;//ultima directie in care am mers(scos sau bagat)
    private Boolean GlSensLastDep = true;//ultima directie in care am mers(sus sau jos)
    double glPwr = 0.8;//putere glisiera
    double crPwr = 0.8;//putere cremaliera
    boolean crMg_OK;//daca senzorul de pe cremaliera simte unul dintre magneti
    boolean glMg_OK;//daca senzorul de pe glisiera simte unul dintre magneti
    boolean prevX = false, prevLeft = false, prevRight = false;
    double speed, robotAngle;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    double ok = 0;
    double power = 1.5;//1.41
    double FranaS, FranaD;
    boolean leftstickpress; //Outside of loop()
    double intakePower = 0;
    boolean intakeOk = true;
    int target = 0;
    int intakePowerR = 0;
    boolean intakeOkR = true;
    boolean changed = false;
    DcMotor TleftDrive = null;
    DcMotor TrightDrive = null;
    DcMotor BleftDrive = null;
    DcMotor BrightDrive = null;
    final double TICKS_PER_REV = 537.6 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference 9.6
    double TICKS_PER_CM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    MecanumDrive m_robotDrive;
    private GamepadEx driverOp;
    private GamepadEx uselessOp;
    BNO055IMU imu;
    private Orientation angles;
    public DistanceSensor sensorRange;
    Rev2mDistanceSensor sensorDistance;
    /**
     *
     */


        //Declare motor objects, gamepad, and drive
        private Motor fL, fR, bL, bR;
        private MecanumDrive m_drive;

    @Override
    public void init()  {
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorDistance= (Rev2mDistanceSensor)sensorRange;

        TleftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        BleftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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
        sensorDistance.initialize();}


        public void loop() {
            // run until the end of the match (driver presses STOP)
            //WRealeaseLeftTeleOp();//A si B pentru WRelease
            //SingleShotTeleOp();

            //MultiShotTeleOp();s
AutoPos();
SetIMU();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            // create our drive object
            speed = -Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            TleftDrive.setPower((speed * Math.sin(robotAngle) - gamepad1.right_stick_x) * power);
            TrightDrive.setPower((speed * Math.cos(robotAngle) + gamepad1.right_stick_x) * power);
            BleftDrive.setPower((speed * Math.cos(robotAngle) - gamepad1.right_stick_x) * power);
            BrightDrive.setPower((speed * Math.sin(robotAngle) + gamepad1.right_stick_x) * power);
            //uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot

telemetry.addData("distance",sensorDistance.getDistance(DistanceUnit.CM));
telemetry.update();
            //telemetry.update();


        }

public void AutoPos()
{
    if (gamepad1.a)
    {


        while (sensorDistance.getDistance(DistanceUnit.CM)>68||sensorDistance.getDistance(DistanceUnit.CM)<63)
        {
            if (sensorDistance.getDistance(DistanceUnit.CM)>68)normalstrafeDrive(0.5,5,-5);
            if (sensorDistance.getDistance(DistanceUnit.CM)<63)normalstrafeDrive(0.5,-5,-5);
            telemetry.addData("we in AutoPos",1);
            telemetry.addData("distance",sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        //m_robotDrive.stop();

        while (angles.firstAngle>2.5||angles.firstAngle<-2.5){
            if (angles.firstAngle>2.5)normalDrive(0.5,2,-2);
            if (angles.firstAngle<-2.5)normalDrive(0.5,2,-2);
            telemetry.addData("we in AutoPos",2);
            telemetry.addData("angle",angles.firstAngle);
            telemetry.update();
        }

        TleftDrive.setPower(0);
        TrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }
}
public void SetIMU()
{
    if (gamepad1.b)
    {BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
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

}

    public void normalDrive(double speed, double leftInches, double rightInches){

// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = TleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = TrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        TleftDrive.setTargetPosition(newLeftFrontTarget);
        TrightDrive.setTargetPosition(newRightFrontTarget);
        BrightDrive.setTargetPosition(newRightBackTarget);
        BleftDrive.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converdets it into power for the motor.
        TrightDrive.setPower(Math.abs(speed));
        TleftDrive.setPower(Math.abs(speed));
        BrightDrive.setPower(Math.abs(speed));
        BleftDrive.setPower(Math.abs(speed));
        while(TleftDrive.isBusy() && BleftDrive.isBusy() &&  TrightDrive.isBusy() &&  BrightDrive.isBusy() )
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",TleftDrive.getCurrentPosition());
            telemetry.update();
            //initDiff=frontEncoder.getCurrentPosition()-leftEncoder.getCurrentPosition();
        }








    }
    public void normalstrafeDrive(double speed, double leftInches, double rightInches){

// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BleftDrive.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = TleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = TrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        // this flips the diagonals which allows the robot to strafe
        TleftDrive.setTargetPosition(-newLeftFrontTarget);
        TrightDrive.setTargetPosition(-newRightFrontTarget);
        BrightDrive.setTargetPosition(newRightBackTarget);
        BleftDrive.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        TrightDrive.setPower(Math.abs(speed));
        TleftDrive.setPower(Math.abs(speed));
        BrightDrive.setPower(Math.abs(speed));
        BleftDrive.setPower(Math.abs(speed));


        while(TleftDrive.isBusy() && BleftDrive.isBusy() &&  TrightDrive.isBusy() &&  BrightDrive.isBusy())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",TleftDrive.getCurrentPosition());
            telemetry.update();
        }



    }
}


