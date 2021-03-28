package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
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

    static final double TRACKWIDTH = 17.7 ;
    static final double WHEEL_DIAMETER = 6.0/2.54;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 17/2.54;

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
        fL.setInverted(true);
        fR.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);


        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(hardwareMap, "frontEncoder");


        //leftEncoder.setInverted(true);
        //rightEncoder.setInverted(true);
        centerEncoder.setInverted(true);

        // calculate multiplier
        //TICKS_TO_INCHES = 434;
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / 8192;
        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        DriveThread OdoThread = new DriveThread();
        OdoThread.start();

        m_odometry = new OdometrySubsystem(m_robotOdometry);

        Waypoint p1 = new StartWaypoint(0, 0);

//Waypoint p2 = new GeneralWaypoint(40,0,Math.toRadians(0),0.8,0.8,30);
//Waypoint p3 = new GeneralWaypoint(40,40);
        Waypoint p4 = new EndWaypoint(50,0,Math.toRadians(0),0.8,-0.8,30,40,Math.toRadians(10));

        // we are using the waypoints we made in the above examples
        Path m_path = new Path(p1,p4);
int n = 0;
        m_path.init(); // initialize the path

        waitForStart();




while (opModeIsActive())
{


m_path.followPath(m_robotDrive, m_robotOdometry);
if (m_path.isFinished()){break;}
}
OdoThread.interrupt();;


    }

    public class DriveThread extends Thread
    {
        public DriveThread()
        {
            this.setName("DriveThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {



                while (!isInterrupted()) {
                    m_robotOdometry.updatePose();
                    telemetry.addData("x&y : ",m_robotOdometry.getPose());
                    telemetry.update();

                }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.


            //Logging.log("end of thread %s", this.getName());
        }
    }
}



