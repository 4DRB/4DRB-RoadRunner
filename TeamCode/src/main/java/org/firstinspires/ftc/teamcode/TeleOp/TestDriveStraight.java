package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="TestStraight", group="Linear Opmode")
@Disabled

public class TestDriveStraight extends LinearOpMode {
    private int speedAdjust = 8;
    boolean isPulling = false, clamped = false, prevX = false, prevLeft = false, prevRight = false, prevB = false, rotated = false, prevA = false;
    double speed, robotAngle, realign = 0;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;
    public double offTLD=3.25;
    public double offBLD=2.25;
    public double offTRD=4;
    public double offBRD=6.5;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    /*private Servo LegoStanga1 = null;
    private  Servo LegoStanga2 = null;
    private Servo LegoDreapta1 = null;
    private  Servo LegoDreapta2 = null;
    private  Servo Grip = null;
    private DcMotor ColectorE = null;
    private DcMotor ColectorV = null;
    private DcMotor CulisataOrizontala=null;
    private DcMotor CulisantaVerticala=null;
    DigitalChannel  TouchDreapta;
    DigitalChannel  TouchStanga;*/
    double ok=0;
    double okOff=0;
    double  power   = 0.3;
    double FranaS,FranaD;
    // public Servo ServomotorE = null;
    //public Servo ServomotorV = null;
    double k=0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        //leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
       // leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        //rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        //rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        /*ColectorE = hardwareMap.get(DcMotor.class, "ColectorE");
        ColectorV = hardwareMap.get(DcMotor.class, "ColectorV");
        CulisataOrizontala= hardwareMap.get(DcMotor.class, "CulisantaOrizontala");
        CulisantaVerticala= hardwareMap.get(DcMotor.class, "CulisantaVerticala");
        ServomotorE = hardwareMap.servo.get("Servo_SpateE");
        ServomotorV = hardwareMap.servo.get("Servo_SpateV");
        LegoStanga1 = hardwareMap.servo.get("LegoStanga1");
        LegoStanga2 = hardwareMap.servo.get("LegoStanga2");
        LegoDreapta1 = hardwareMap.servo.get("LegoDreapta1");
        LegoDreapta2 = hardwareMap.servo.get("LegoDreapta2");
        Grip=hardwareMap.servo.get("Grip");*/


        //ColectorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ColectorV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //TouchStanga = hardwareMap.get(DigitalChannel.class, "TouchStanga");
        //TouchDreapta = hardwareMap.get(DigitalChannel.class, "TouchDreapta");
        // set the digital channel to input.
        //TouchStanga.setMode(DigitalChannel.Mode.INPUT);
        //TouchDreapta.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(ok==0)
            {TleftDrive.setPower(0);
                BleftDrive.setPower(0);
                TrightDrive.setPower(0);
                BrightDrive.setPower(0);
                ok=1;
            }
            telemetry.update();
            //ColectorE.setPower(0.7);
            //ColectorV.setPower(0.7);

            while (gamepad1.dpad_up) {
                TrightDrive.setPower(-power + power*-offTRD/100);
                BrightDrive.setPower(-power + power*-offBRD/100);
                TleftDrive.setPower(-power + power*-offTLD/100);
                BleftDrive.setPower(-power + power*-offBLD/100);
                telemetry.update();
                if (!gamepad1.dpad_up) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }

            }

            while (gamepad1.dpad_down) {
                /*TleftDrive.setPower(power + power*(offTLD-0.9)/100);
                BleftDrive.setPower(power + power*(offBLD-0.89)/100);
                TrightDrive.setPower(power + power*(offTRD+1.45)/100);
                BrightDrive.setPower(power + power*(offBRD+0.95)/100);*/
                TleftDrive.setPower(power + power*offTLD/100);
                BleftDrive.setPower(power + power*offBLD/100);
                TrightDrive.setPower(power + power*offTRD/100);
                BrightDrive.setPower(power + power*offBRD/100);



                if (gamepad1.start)
                    power = 0.3;
                if (!gamepad1.dpad_down) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }
            }
            telemetry.update();

            while (gamepad1.dpad_left) {
                TleftDrive.setPower(power + power*offTLD/100);
                BleftDrive.setPower(-power + power* -offBLD/100);
                TrightDrive.setPower(-power + power* -offTRD/100);
                BrightDrive.setPower(power + power* offBRD/100);
                if (!gamepad1.dpad_left) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }
            }
            telemetry.update();

            while (gamepad1.dpad_right) {
                TleftDrive.setPower(-power + power*-offTLD/100);
                BleftDrive.setPower(power + power*offBLD/100);
                TrightDrive.setPower(power + power*offTRD/100);
                BrightDrive.setPower(-power + power*-offBRD/100);

                if (!gamepad1.dpad_right) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }
            }


            if (gamepad1.a&&okOff==0)
            {
                offTRD=offTRD+0.1;
                okOff=1;
            }

            if (gamepad1.b&&okOff==0) {
                offTRD=offTRD-0.1;
                okOff=1;

            }

            if (gamepad1.x&&okOff==0) {
                offTLD=offTLD+0.1;
                okOff=1;
            }

            if (gamepad1.y&&okOff==0) {
                offTLD=offTLD-0.1;
                okOff=1;
            }
            if (gamepad1.left_bumper&&okOff==0) {
               offBRD=offBRD+0.1;
                okOff=1;
            }
            if (gamepad1.right_bumper&&okOff==0) {
                offBRD=offBRD-0.1;
                okOff=1;
            }
            if (gamepad1.start&&okOff==0)
            {
                offBLD=offBLD+0.1;
                okOff=1;
            }
            if (gamepad1.back&&okOff==0)
            {
                offBLD=offBLD-0.1;
                okOff=1;
            }
            if(!gamepad1.a&&!gamepad1.b&&!gamepad1.x&&!gamepad1.y&&!gamepad1.left_bumper&&!gamepad1.right_bumper&&!gamepad1.start&&!gamepad1.back)
            {
                okOff=0;
            }
            telemetry.addData("offTRD",offTRD);
            telemetry.addData("offTLD",offTLD);
            telemetry.addData("offBRD",offBRD);
            telemetry.addData("offBLD",offBLD);
            telemetry.update();

            /*while(gamepad2.right_bumper)
            {
                CulisataOrizontala.setPower(0.8);
                if(!gamepad2.right_bumper)
                {
                    CulisataOrizontala.setPower(0);
                    break;
                }
            }
            while(gamepad2.left_bumper)
            {
                CulisataOrizontala.setPower(-0.8);
                if(!gamepad2.left_bumper)
                {
                    CulisataOrizontala.setPower(0);
                    break;
                }
            }
            if(gamepad2.dpad_left)
            {
                Grip.setPosition(-0.3);
            }
            if(gamepad2.dpad_right)
            {
                Grip.setPosition(0.3);
            }
            while(gamepad2.dpad_up)
            {
                if (TouchDreapta.getState() == true) {
                    telemetry.addData("TouchDreapta", "Is Not Pressed");
                    CulisantaVerticala.setPower(-1);
                }
                else
                    telemetry.addData("TouchDreapta", "Is Pressed");
                CulisantaVerticala.setPower(0);
                telemetry.update();
            }
            while(gamepad2.dpad_down)
            {
                if (TouchStanga.getState() == true) {
                    telemetry.addData("TouchStanga", "Is Not Pressed");
                    CulisantaVerticala.setPower(0.8);
                }
                else
                    telemetry.addData("TouchStanga", "Is Pressed");
                CulisantaVerticala.setPower(0);
                telemetry.update();
            }*/
            //controls speed adjusting
            if(gamepad1.dpad_left && !prevLeft) speedAdjust--;
            if(gamepad1.dpad_right && !prevRight) speedAdjust++;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;

            //sets clip that attaches to the foundation

            prevX = gamepad1.x;


            //uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot
            speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            TleftDrive.setPower(speed * Math.sin(robotAngle) - gamepad1.right_stick_x + power*-offTLD/100);
            TrightDrive.setPower(speed * Math.cos(robotAngle) + gamepad1.right_stick_x + power*offTLD/100);
            BleftDrive.setPower(speed * Math.cos(robotAngle) - gamepad1.right_stick_x + power*-offTLD/100);
            BrightDrive.setPower(speed * Math.sin(robotAngle) + gamepad1.right_stick_x + power*offTLD/100);
            telemetry.update();

        }
    }

    /*public void CurbaFata(double power,double FranaD,double FranaS){
        FranaD=gamepad1.right_trigger;
        FranaS=gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(-(power - FranaS));
            BleftDrive.setPower(-(power - FranaS));
        }
        if(gamepad1.right_trigger!=0){
            TrightDrive.setPower(-(power-FranaD));
            BrightDrive.setPower(-(power-FranaD));
        }

    }
    public void CurbaSpate(double power,double FranaD,double FranaS){
        FranaD=gamepad1.right_trigger;
        FranaS=gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(power - FranaS);
            BleftDrive.setPower(power - FranaS);
        }
        if(gamepad1.right_trigger!=0){
            TrightDrive.setPower(power-FranaD);
            BrightDrive.setPower(power-FranaD);
        }

    }*/
}