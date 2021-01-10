package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TestController")
public class Test_Controller extends LinearOpMode {

    private Boolean CrSensLastDep = true;//ultima directie in care am mers(scos sau bagat)
    private Boolean GlSensLastDep = true;//ultima directie in care am mers(sus sau jos)
    double glPwr = 0.8;//putere glisiera
    double crPwr = 1.0;//putere cremaliera
    boolean crMg_OK;//daca senzorul de pe cremaliera simte unul dintre magneti
    boolean glMg_OK;//daca senzorul de pe glisiera simte unul dintre magneti

    /**
     *
     */
    @Override
    public void runOpMode() {
        Servo clamp = hardwareMap.get(Servo.class, "SR_CLAMP");
        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel digChannel = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        // set the digital channel to input.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        digChannel.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)
            WRealeaseLeftTeleOp();//A si B pentru WRelease
            SingleShotTeleOp();

            MultiShotTeleOp();

            ClampTeleOp();//X si Y pentru clamp


            CremalieraTeleOp();//sageti stanga dreapta


            GlisieraTeleOp();//Sageti sus si jos

            IntakeTeleOp();//LeftStickBumper




            telemetry.addData("sens", CrSensLastDep);
            telemetry.addData("magnet", crMg_OK);
            telemetry.addData("sageata dreapta", gamepad1.dpad_right);
            telemetry.addData("sageata stanga", gamepad1.dpad_left);
            telemetry.addData("sens glisiera", GlSensLastDep);
            telemetry.addData("magnet glisiera", glMg_OK);
            telemetry.addData("Sageata sus", gamepad1.dpad_up);
            telemetry.addData("sageata jos", gamepad1.dpad_down);
            telemetry.update();
        }
    }

    // run until the end of the match (driver presses STOP)
    public void WRealeaseLeftTeleOp() {
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");


        if (gamepad1.a) {
            // move to 0 degrees.
            wRelease.setPosition(0.0);

        } else if (gamepad1.b) {
            // move to 90 degrees.
            wRelease.setPosition(0.3);
        }
        telemetry.addData("Servo Position", wRelease.getPosition());
        telemetry.addData("Status", "Running");
    }
    public void SingleShotTeleOp(){
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.back){
            Launcher1.setPower(power);
            Launcher2.setPower(power);
            sleep(1000);
            Shooter.setPosition(0.3);
            sleep(300);
            Shooter.setPosition(0);
        } else {
            Launcher1.setPower(0);
            Launcher2.setPower(0);}
    }
    public void MultiShotTeleOp(){
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.start){
            Launcher1.setPower(power);
            Launcher2.setPower(power);
            sleep(1000);
            Shooter.setPosition(0.3);
            sleep(300);
            Shooter.setPosition(0);
            sleep(300);
            Shooter.setPosition(0.3);
            sleep(300);
            Shooter.setPosition(0);
            sleep(300);
            Shooter.setPosition(0.3);
            sleep(300);
            Shooter.setPosition(0);
            sleep(300);

        }


    }
    public void IntakeTeleOp(){
        double power = 1;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class,"leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.left_stick_button){
            InTake.setPower(power);
        } else {
            InTake.setPower(0);
        }
    }

    public void ClampTeleOp() {
        Servo clamp = hardwareMap.get(Servo.class, "SR_CLAMP");


        if (gamepad1.y) {
            // move to 0 degrees.
            clamp.setPosition(0.0);

        } else if (gamepad1.x) {
            // move to 90 degrees.
            clamp.setPosition(0.3);
        }
        telemetry.addData("Servo Position", clamp.getPosition());
        telemetry.addData("Status", "Running");
    }



    public void CremalieraTeleOp() {

        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel digChannel = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        crMg_OK = !digChannel.getState();
        if (CrSensLastDep) {
            if (crMg_OK) {
                if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(0);
                    //CrSensLastDep = false;
                } else if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(-crPwr);
                    //CrSensLastDep = false;
                }

            } else {
                if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    ////////////////////////CrSensLastDep = true;
                } else if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(0);
                    CrSensLastDep = false;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        } else {
            if (crMg_OK) {
                if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(0);
                } else if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    //CrSensLastDep = true;
                }
            }
            if (!crMg_OK) {
                if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(-crPwr);
                } else if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    CrSensLastDep = true;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        }


    }

    public void GlisieraTeleOp() {

        DcMotor glisiera = hardwareMap.get(DcMotorEx.class, "GLS");
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DigitalChannel crMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");
        glMg_OK = !crMg.getState();
        glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (GlSensLastDep) {
            if (glMg_OK) {
                if (gamepad1.dpad_down) {
                    glisiera.setPower(0);

                    //CrSensLastDep = false;
                } else if (gamepad1.dpad_up) {
                    glisiera.setPower(glPwr);
                    //CrSensLastDep = false;
                }

            } else {
                if (gamepad1.dpad_down) {
                    glisiera.setPower(-glPwr);
                    ////////////////////////CrSensLastDep = true;
                } else if (gamepad1.dpad_up) {
                    glisiera.setPower(0);
                    GlSensLastDep = false;
                } else {
                    glisiera.setPower(0);
                }
            }
        } else {
            if (glMg_OK) {
                if (gamepad1.dpad_up) {
                    glisiera.setPower(0);
                } else if (gamepad1.dpad_down) {
                    glisiera.setPower(-glPwr);
                    //CrSensLastDep = true;
                }
            }
            if (!glMg_OK) {
                if (gamepad1.dpad_up) {
                    glisiera.setPower(glPwr);
                } else if (gamepad1.dpad_down) {
                    glisiera.setPower(-glPwr);
                    GlSensLastDep = true;
                } else {
                    glisiera.setPower(0);
                }
            }
        }

    }
}

