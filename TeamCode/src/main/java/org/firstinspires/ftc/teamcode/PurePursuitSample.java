package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous

public class PurePursuitSample extends CommandOpMode {

    // define our constants
    static final double TRACKWIDTH = 17.7 ;
    static final double WHEEL_DIAMETER = 6.0/2.54;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 17/2.54;

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void initialize() {
        fL = new MotorEx(hardwareMap, "FL", MotorEx.GoBILDA.RPM_435);
        fR = new MotorEx(hardwareMap, "FR", MotorEx.GoBILDA.RPM_435);
        bL = new MotorEx(hardwareMap, "BL", MotorEx.GoBILDA.RPM_435);
        bR = new MotorEx(hardwareMap, "BR", MotorEx.GoBILDA.RPM_435);
        fL.setInverted(true);
        fR.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);
        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);
//m_robotDrive.setRightSideInverted(true);

        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(hardwareMap, "frontEncoder");


        // calculate multiplier
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / 8192;

        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        m_odometry = new OdometrySubsystem(m_robotOdometry);

        // create our pure pursuit command
        ppCommand = new PurePursuitCommand(
                m_robotDrive, m_odometry,
                new StartWaypoint(0, 0),
                new PointTurnWaypoint(50,0,Math.toRadians(90),0.8,0.5,30,100,Math.toRadians(3600)),
               // new PointTurnWaypoint(50,50,Math.toRadians(90),0.8,0.5,30,10,10),
                new EndWaypoint(
                        50, 0, Math.toRadians(90), 0.5,
                        0.5, 30, 100, Math.toRadians(360)
                )
        );

        // schedule the command
        schedule(ppCommand);
        while (!ppCommand.isFinished()&&opModeIsActive())
        {
            telemetry.addData("x&y : ",m_robotOdometry.getPose());
            telemetry.update();
        }
    }


}



