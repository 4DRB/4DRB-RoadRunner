package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.actions.InterruptAction;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
//@Disabled
public class TestPP extends LinearOpMode {

    // define our constants

    static final double TRACKWIDTH = 17.7;
    static final double WHEEL_DIAMETER = 6.0*2.54;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 2.4;

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private Subsystem m_stopsystem;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void runOpMode() {
        fL = new MotorEx(hardwareMap, "FL", MotorEx.GoBILDA.RPM_435);
        fR = new MotorEx(hardwareMap, "FR", MotorEx.GoBILDA.RPM_435);
        bL = new MotorEx(hardwareMap, "BL", MotorEx.GoBILDA.RPM_435);
        bR = new MotorEx(hardwareMap, "BR", MotorEx.GoBILDA.RPM_435);
        //fR.setInverted(true);
        //bR.setInverted(true);

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(hardwareMap, "frontEncoder");

        // calculate multiplier
        //TICKS_TO_INCHES = 434;
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / leftEncoder.getCPR();
        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
InterruptAction stopAction = new InterruptAction() {
    @Override
    public void doAction() {
        m_robotDrive.stop();
        stop();
    }
};
        m_odometry = new OdometrySubsystem(m_robotOdometry);

        Waypoint p1 = new StartWaypoint(0, 0);

        Waypoint p5 = new InterruptWaypoint(
                50,50,0.2,0.2,50,50,Math.toRadians(359),stopAction
        );
        Waypoint p3 = new EndWaypoint(50,50,0.2,0.2,0.2,7,8,9);

        // we are using the waypoints we made in the above examples
        Path m_path = new Path(p1, p5,p3);

        //m_path.init(); // initialize the path

        waitForStart();

            m_path.followPath(m_robotDrive, m_robotOdometry);
            m_robotOdometry.updatePose();

    }


}