package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name= "TestServoSensorMagnetic")
public class Test_Servo_Sensori_Magnetici extends LinearOpMode {

    //public DcMotor MotorDF = null;
    //public DcMotor MotorSF = null;
    //public DcMotor MotorDJ = null;
    //public DcMotor MotorSJ = null;
    //private DcMotor ColectorE = null;
    //private DcMotor ColectorV = null;
    private CRServo Cremaliera_Servo;
    private DigitalChannel DigChannel;
    DigitalChannel digitalTouch;  // Hardware Device Object

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        Cremaliera_Servo = hardwareMap.get(CRServo.class, "Cremaliera");

        DigChannel = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        // set the digital channel to input.
        DigChannel.setMode(DigitalChannel.Mode.INPUT);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //am configurat gyroul integrat si i-am definit unitatea de masura
        int Incrementer = 0;
        while(DigChannel.getState()!=false)
        {

            Cremaliera_Servo.setPower(1);
        }
        Cremaliera_Servo.close();




    }
}
