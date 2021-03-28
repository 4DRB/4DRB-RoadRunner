package treamcode;

//import com.qualcomm.robotcore.util.Range;

import static com.company.Robot.*;
//import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import com.company.FloatPoint;
import com.company.Range;

import org.opencv.core.Point;

import java.util.ArrayList;

import static treamcode.MathFunctions.AngleWrap;
import static RobotUtilities.MovementVars.*;
import static treamcode.MathFunctions.lineCircleIntersection;
import static treamcode.MyOpMode.WorldX;
import static treamcode.MyOpMode.WorldY;

public class RobotMovement {
    public static double distanceToTarget2;
    public static double distanceToX2;
    public static double distanceToY2;
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){


        CurvePoint followMe = getFollowPointPath(allPoints,new Point(worldXPosition,worldYPosition),allPoints.get(0).followDistance);


        goToPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000000;

            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        //int worldYPosition;
        //int worldXPosition;
         double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        double distanceToX = x - worldXPosition;
        double distanceToY = y - worldYPosition;
        distanceToX2 = distanceToX;
        distanceToY2 = distanceToY;
distanceToTarget2 = distanceToTarget;
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 10) {
            movement_turn = 0;
        }
        if (Math.abs(distanceToX)-Math.abs(WorldX())<10&&Math.abs(distanceToY)-Math.abs(WorldY())<10)
        {
            movement_x = 0;
            movement_y = 0;}


    }
    public static double leechX()
    {
        return movement_x;
    }
    public static double leechY()
    {
        return movement_y;
    }
    public static double leechTurn()
    {
        return movement_turn;
    }
    public static double leechDistance(){ return distanceToTarget2;}
    public static double leechDistanceX(){ return distanceToX2;}
    public static double leechDistanceY(){ return distanceToY2;}
}
