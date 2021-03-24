package treamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

import static treamcode.RobotMovement.distanceToTarget;
import static treamcode.RobotMovement.followCurve;
import static treamcode.RobotMovement.leechTurn;
import static treamcode.RobotMovement.leechX;
import static treamcode.RobotMovement.leechY;

@TeleOp(name = "Bwaaaah sutututututu")

public class MyOpMode extends OpMode {
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;

    double speed, robotAngle,power=0.5;
    @Override
    public void init() {
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
    }

    @Override
    public void loop() {

        //RobotMovement.goToPosition(358/2,358/2,0.3,Math.toRadians(90),0.3);
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        //allPoints.add(new CurvePoint(0,0.0,1.0,1.0,50.0,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(50,0,1.0,1.0,30,Math.toRadians(180),1.0));
        /*allPoints.add(new CurvePoint(-150,20,1.0,1.0,30,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(100,10,1.0,1.0,30,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(10,5,1.0,1.0,50.0,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(180,0.0,1.0,1.0,50.0,Math.toRadians(50),1.0));
*/
        followCurve(allPoints,Math.toRadians(180));

double x = leechX();
double y = leechY();
double turn = leechTurn();
///////////////////////////////////////////////////////ENCODERELE FA TEST VALORILE SUNT CACA
        speed = -Math.hypot(x, y);

        //finds the angle the robot is moving at
        //robotAngle = Math.atan2(y, x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
        robotAngle = Math.atan2(y, x) - Math.PI / 4;
        //finds the percent of power to each wheel and multiplies it by the speed
        TleftDrive.setPower((speed * Math.sin(robotAngle) - turn) * power);
        TrightDrive.setPower((speed * Math.cos(robotAngle) + turn) * power);
        BleftDrive.setPower((speed * Math.cos(robotAngle) - turn) * power);
        BrightDrive.setPower((speed * Math.sin(robotAngle) + turn) * power);


        telemetry.addData("x = ",x);
        telemetry.addData("y = ",y);
        telemetry.addData("turn = ",turn);
        telemetry.addData("distance to target = ",distanceToTarget);
    }
}
