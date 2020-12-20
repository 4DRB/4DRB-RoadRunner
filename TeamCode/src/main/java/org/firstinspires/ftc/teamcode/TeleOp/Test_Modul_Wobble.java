package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name= "TestModulWobble")
public class Test_Modul_Wobble extends LinearOpMode {

    private Boolean CrSensLastDep = true;

    /**
     *
     */
    @Override
    public void runOpMode() {


        //public DcMotor MotorDF = null;
        //public DcMotor MotorSF = null;
        //public DcMotor MotorDJ = null;
        //public DcMotor MotorSJ = null;
        //private DcMotor ColectorE = null;
        //private DcMotor ColectorV = null;
        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel digChannel = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        Servo clamp=hardwareMap.get(Servo.class, "SR_CLAMP");
        // set the digital channel to input.
        digChannel.setMode(DigitalChannel.Mode.INPUT);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        waitForStart();
        while (opModeIsActive()) {


            // run until the end of the match (driver presses STOP)


                // check to see if we need to move the servo.
                if(gamepad1.y) {
                    // move to 0 degrees.
                    clamp.setPosition(0.0);

                } else if (gamepad1.x) {
                    // move to 90 degrees.
                    clamp.setPosition(0.5);
                }
                telemetry.addData("Servo Position", clamp.getPosition());
                telemetry.addData("Status", "Running");





            boolean crMg_OK;
            /*if (CrSensLastDep == false) {
                    CrInt_OK = false;
                    CrExt_OK = true;
                } else if (CrSensLastDep == true) {
                    CrExt_OK = false;
                    CrInt_OK = true;
                }*/
            /* if (CrPw_0 == false) {
                    Cremaliera_Servo.setPower(0);
                    CrPw_0 = true;
                }*/
            crMg_OK = !digChannel.getState();


                // Buton A pentru cremaliera in exterior = True
                // Buton B pentru cremaliera in interior = False
                // CrSensDep = 0 deplaseaza cremaliera in interior
                // CrSensDep = 1 deplaseaza cremaliera in exterior
                /*if (gamepad1.b) {
                    if (CrSensLastDep == false ) {
                        if (CrMg_OK == false) {
                            Cremaliera_Servo.setPower(-1);
                            CrSensLastDep = false;
                        }

                    }
                        else if (CrSensLastDep == true)
                        {
                            if (CrMg_OK == true) {
                                Cremaliera_Servo.setPower(0);
                                CrSensLastDep = false;
                            }
                        }
                    }
                    // move to 90 degrees.
                    /*if (CrInt_OK == true) {
                        Cremaliera_Servo.setPower(-1);
                        CrSensDep = false;
                        CrPw_0 = false;
                    }
                 else if (gamepad1.a) {
                    if (CrSensLastDep == true )
                    {
                        if (CrMg_OK == false)
                        {
                            Cremaliera_Servo.setPower(-1);
                            CrSensLastDep = true;
                        }
                    }
                    // move to 180 degrees.
                    if (CrExt_OK == true) {
                        Cremaliera_Servo.setPower(1);
                        CrSensDep = true;
                        CrPw_0 = false;
                    }
            } else if (gamepad1.y) {
                    // move to 180 degrees.
                    Cremaliera_Servo.setPower(0);
                }*/
                // 1100-0
                // 0101-0
                //0100-0

                if (CrSensLastDep)
                {   if (crMg_OK)
                    {
                        if (gamepad1.a)
                        {
                            cremaliera_Servo.setPower(0);
                            //CrSensLastDep = false;
                        }
                        else if (gamepad1.b)
                        {
                            cremaliera_Servo.setPower(-1);
                            //CrSensLastDep = false;
                        }

                    }
                    else {
                        if (gamepad1.a)
                        {
                            cremaliera_Servo.setPower(1);
                            ////////////////////////CrSensLastDep = true;
                        }
                        else if(gamepad1.b){
                            cremaliera_Servo.setPower(0);
                            CrSensLastDep =false;
                        }
                        else{
                            cremaliera_Servo.setPower(0);
                        }
                    }
                }
                else {
                    if (crMg_OK) {
                        if (gamepad1.b)
                        {
                            cremaliera_Servo.setPower(0);
                        }
                        else if (gamepad1.a)
                        {
                            cremaliera_Servo.setPower(1);
                            //CrSensLastDep = true;
                        }
                    }
                    if (!crMg_OK) {
                        if (gamepad1.b)
                        {
                            cremaliera_Servo.setPower(-1);
                        }
                        else if (gamepad1.a){
                            cremaliera_Servo.setPower(1);
                            CrSensLastDep = true;
                        }
                        else {
                            cremaliera_Servo.setPower(0);
                        }
                    }
                }
            telemetry.addData("sens", CrSensLastDep);
            telemetry.addData("magnet",crMg_OK);
            telemetry.addData("Buton A", gamepad1.a);
            telemetry.addData("Buton B", gamepad1.b);
            telemetry.update();


            }
        }
        // run until the end of the match (driver presses STOP)


    }

