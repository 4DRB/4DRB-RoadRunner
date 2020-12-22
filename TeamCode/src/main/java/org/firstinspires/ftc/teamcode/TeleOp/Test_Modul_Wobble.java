package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TestModulWobble")
public class Test_Modul_Wobble extends LinearOpMode {

    private Boolean CrSensLastDep = true;

    boolean crMg_OK;


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
            ClampTeleOp();//X si Y pentru clamp
            CremalieraTeleOp();//A si B pentru Cremaliera
        }
    }

    // run until the end of the match (driver presses STOP)
    public void ClampTeleOp(){
        Servo clamp = hardwareMap.get(Servo.class, "SR_CLAMP");


        if (gamepad1.y) {
            // move to 0 degrees.
            clamp.setPosition(0.0);

        } else if (gamepad1.x) {
            // move to 90 degrees.
            clamp.setPosition(0.25);
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
                if (gamepad1.a) {
                    cremaliera_Servo.setPower(0);
                    //CrSensLastDep = false;
                } else if (gamepad1.b) {
                    cremaliera_Servo.setPower(-1);
                    //CrSensLastDep = false;
                }

            } else {
                if (gamepad1.a) {
                    cremaliera_Servo.setPower(1);
                    ////////////////////////CrSensLastDep = true;
                } else if (gamepad1.b) {
                    cremaliera_Servo.setPower(0);
                    CrSensLastDep = false;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        } else {
            if (crMg_OK) {
                if (gamepad1.b) {
                    cremaliera_Servo.setPower(0);
                } else if (gamepad1.a) {
                    cremaliera_Servo.setPower(1);
                    //CrSensLastDep = true;
                }
            }
            if (!crMg_OK) {
                if (gamepad1.b) {
                    cremaliera_Servo.setPower(-1);
                } else if (gamepad1.a) {
                    cremaliera_Servo.setPower(1);
                    CrSensLastDep = true;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        }
        telemetry.addData("sens", CrSensLastDep);
        telemetry.addData("magnet", crMg_OK);
        telemetry.addData("Buton A", gamepad1.a);
        telemetry.addData("Buton B", gamepad1.b);
        telemetry.update();
    }

}

