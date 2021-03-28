package treamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;

import static treamcode.RobotMovement.distanceToTarget2;
import static treamcode.RobotMovement.followCurve;
import static treamcode.RobotMovement.leechDistance;
import static treamcode.RobotMovement.leechDistanceX;
import static treamcode.RobotMovement.leechDistanceY;
import static treamcode.RobotMovement.leechTurn;
import static treamcode.RobotMovement.leechX;
import static treamcode.RobotMovement.leechY;

@TeleOp(name = "Bwaaaah sutututututu")

public class MyOpMode extends OpMode {
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;


    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private Subsystem m_stopsystem;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    double speed, robotAngle,power=-0.5;
    static double XOdo,YOdo;
    BNO055IMU imu;
    static final double TRACKWIDTH = 17.7*2.54 ;
    static final double WHEEL_DIAMETER = 6.0;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 17;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    int n=1;
    @Override
    public void init() {
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / 8192;

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
        TleftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        BleftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //TleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //BleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        //leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);


    }

    @Override
    public void loop() {

        //RobotMovement.goToPosition(358/2,358/2,0.3,Math.toRadians(90),0.3);
        XOdo =leftEncoder.getCurrentPosition()*TICKS_TO_INCHES;
        YOdo =frontEncoder.getCurrentPosition()*TICKS_TO_INCHES;
        //ArrayList<CurvePoint> allPoints = new ArrayList<>();
        //allPoints.add(new CurvePoint(0,0.0,1.0,1.0,50.0,Math.toRadians(50),1.0));
        //allPoints.add(new CurvePoint(50,0,1.0,1.0,30,0,0));
        //allPoints.add(new CurvePoint(50,50,1.0,1.0,30,0,0));
        /*allPoints.add(new CurvePoint(-150,20,1.0,1.0,30,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(100,10,1.0,1.0,30,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(10,5,1.0,1.0,50.0,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(180,0.0,1.0,1.0,50.0,Math.toRadians(50),1.0));
*/
        //followCurve(allPoints,Math.toRadians(180));

        RobotMovement.goToPosition(50,50,1,Math.toRadians(90),1);

double x = leechX();
double y = leechY();
double turn = leechTurn();
///////////////////////////////////////////////////////ENCODERELE FA TEST VALORILE SUNT CACA
        speed = -Math.hypot(x, y);

        //finds the angle the robot is moving at
        //robotAngle = Math.atan2(y, x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
        robotAngle = Math.atan2(y, x) - Math.PI / 4;
        //finds the percent of power to each wheel and multiplies it by the speed
        /*TleftDrive.setPower((speed * Math.sin(robotAngle) - turn) * power);
        TrightDrive.setPower((speed * Math.cos(robotAngle) + turn) * power);
        BleftDrive.setPower((speed * Math.cos(robotAngle) - turn) * power);
        BrightDrive.setPower((speed * Math.sin(robotAngle) + turn) * power);*/

        TleftDrive.setPower((y+x+turn) * power);//1
        TrightDrive.setPower((y-x-turn) * power);//1
        BleftDrive.setPower((y-x-turn) * power);//-1
        BrightDrive.setPower((y+x+turn) * power); //0

n+=1;
        telemetry.addData("x = ",WorldX());
        telemetry.addData("y = ",WorldY());
        telemetry.addData("angle = ",WorldAngle());
        telemetry.addData("Ex = ",x);
        telemetry.addData("Ey = ",y);
        telemetry.addData("Eturn ",turn);
        telemetry.addData("distanceY",leechDistanceY());
        telemetry.addData("distance to target = ",leechDistance());
        telemetry.update();
    }


    public static double WorldX()
    {

        return XOdo;}
    public static double WorldY()
    {
        return YOdo;}
    public double WorldAngle()
    { angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Math.toRadians(angles.firstAngle);}
}
