package org.firstinspires.ftc.teamcode.SeansMotionController.Control;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeanDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Angle;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.LineSegment;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Point;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Pose;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.StateChange;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.StopWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Wait;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Waypoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sean Cardosi on 9/3/22.
 */
public class MotionController {

    SeanDrivetrain drive;
    Localizer localizer;
    final private double DRIVE_P = 0.035;//0.07;//0.1;//0.03;
    final private double DRIVE_I = 0.0;//-0.00006;//0.0001;//0.000001;
    final private double DRIVE_D = 0.50;//0.57;//0.9;//0.04;
    final private double TURN_P = 0.05;//0.03;//0.1;//DRIVE_P;//0.01;
    final private double TURN_I = 0.0;//0.0;//0.0001;//DRIVE_I;//0.0;//0;
    final private double TURN_D = 0.37;//0.1;//0.4;//DRIVE_D;//0.0;//0.0;
    SeansSynchronousPID x;
    SeansSynchronousPID y;
    SeansSynchronousPID r;
    List<LynxModule> allHubs;

    public MotionController(HardwareMap hardwareMap) {
        drive = new SeanDrivetrain(hardwareMap);
        drive.runUsingEncoders();
        localizer = new Localizer(hardwareMap, new Pose(0,0, 0));
        x = new SeansSynchronousPID(DRIVE_P,DRIVE_I,DRIVE_D);
        y = new SeansSynchronousPID(DRIVE_P,DRIVE_I,DRIVE_D);
        r = new SeansSynchronousPID(TURN_P,TURN_I,TURN_D);
        x.setOutputRange(-100.0,100.0);
        y.setOutputRange(-100.0,100.0);
        r.setOutputRange(-100.0,100.0);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }


    public Pose relativeDistanceToPoint(Pose robot, Point target) {
        double rx = robot.getX();
        double ry = robot.getY();
        double distance = target.distance(new Point(rx, ry));
        double relAngle = new Point(rx,ry).minus(target).atan() - robot.getHeading();
        double relX = distance * Math.cos(relAngle);
        double relY = distance * Math.sin(relAngle);
        return new Pose(relX, relY, relAngle);
    }

    /**
     * Runs to a waypoint. Does not consider velocity, PID, or curvature.
     * @param target
     */
    public void simpleRunToPoint(Waypoint target, double PIDActivationDistance) {
//        for (LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }
//        localizer.updatePose(telemetry);
        Pose robotPose = localizer.getPose();
        Pose relTarget = relativeDistanceToPoint(robotPose, target);
        double forwardAngle = target.minus(asPoint(robotPose)).atan();
        double backwardAngle = forwardAngle + Math.PI;
        double angleToForward = Angle.angleWrap(forwardAngle - robotPose.getHeading());
        double angleToBackward = Angle.angleWrap(backwardAngle - robotPose.getHeading());
        double continuousAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
        double desiredAngle = target instanceof HeadingControlledWaypoint ? ((HeadingControlledWaypoint) target).getTargetHeading(): continuousAngle;
        Pose p = new Pose(relTarget.getX(), relTarget.getY(), Angle.angleWrap(desiredAngle - robotPose.getHeading()));

        double distance = Math.abs(asPoint(robotPose).distance(asPoint(target)));
//        double xPower = x.calculateUseError(p.getX() * distance);
//        double yPower = y.calculateUseError(p.getY() * distance);
//        double rPower = r.calculateUseError(p.getHeading() * (distance + PIDActivationDistance));
        drive.setMotorPowers(-p.getX(), -p.getY(), p.getHeading() * (distance + PIDActivationDistance), target.speed);
    }

    /**
     * Runs to a waypoint using PID
     * @param target
     */
    public void advancedRunToPoint(Waypoint target, double PIDActivationDistance, LinearOpMode opMode, boolean holdTarget) {
        double distance = 0;
        boolean onTarget = false;
        boolean timesUp = false;

        ElapsedTime eTime = new ElapsedTime();
        eTime.reset();
        do {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            localizer.updatePose();
            Pose robotPose = localizer.getPose();
            Pose relTarget = relativeDistanceToPoint(robotPose, target);
            double forwardAngle = target.minus(asPoint(robotPose)).atan();
            double backwardAngle = forwardAngle + Math.PI;
            double angleToForward = Angle.angleWrap(forwardAngle - robotPose.getHeading());
            double angleToBackward = Angle.angleWrap(backwardAngle - robotPose.getHeading());
            double continuousAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
            double desiredAngle = target instanceof HeadingControlledWaypoint ? ((HeadingControlledWaypoint) target).getTargetHeading() : continuousAngle;
            distance = Math.abs(asPoint(robotPose).distance(asPoint(target)));
            Pose p = new Pose(relTarget.getX(), relTarget.getY(), (Angle.angleWrap(desiredAngle - robotPose.getHeading())));

            double xPower = x.calculateUseError(p.getX());
            double yPower = y.calculateUseError(p.getY());
            double rPower = r.calculateUseError(p.getHeading() * (distance + PIDActivationDistance));

            drive.setMotorPowers(-xPower, -yPower, rPower, target.speed);

            double holdTime = 0.1;//TODO: Find a reasonable value for this
            if (holdTarget) {
                holdTime = 0.2;//0.25
            }

//            onTarget = distance < 1.0 && Math.abs(p.getHeading()) < Math.toRadians(1);
            onTarget = Math.abs(drive.getMaxMotorPower()) < 0.07;//TODO: Find a reasonable value that enables both accuracy and speed
//            if (!(target instanceof StopWaypoint)){
//                onTarget = distance < 4.5 && Math.abs(p.getHeading()) < Math.toRadians(5);
//            }

//            if (target instanceof Wait) {//TODO: Maybe do this
//                holdTime = ((Wait) target).waitTime;
//            }

            if (!onTarget) {
                eTime.reset();
            }
            if (onTarget && eTime.seconds() > holdTime) {
                timesUp = true;
            }
        } while (!opMode.isStopRequested() && !timesUp);
        drive.setMotorPowers(0,0,0,0);
    }

    /**
     * Follows a provided path.
     * @param path An ArrayList of different types of Waypoints
     * @param PIDActivationDistance The distance in centimeters from the target to start using PID
     * @param opMode Usually "this"
     */
    public void followPath(ArrayList<Waypoint> path, double PIDActivationDistance, LinearOpMode opMode) {
        int waypointIndex = 0;
        boolean resetTimer = false;
        ElapsedTime waitTime = new ElapsedTime();
        while (waypointIndex < path.size() && !opMode.isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            localizer.updatePose();
            Pose robot = localizer.getPose();
            Waypoint currentWaypoint = path.get(waypointIndex);
            if (currentWaypoint instanceof StateChange) {
                ((StateChange) currentWaypoint).thread.setState(((StateChange) currentWaypoint).state);
                waypointIndex++;
                continue;
            }
            if (currentWaypoint instanceof Wait) {
                if (!resetTimer) {
                    waitTime.reset();
                    resetTimer = true;
                }
                if (waitTime.milliseconds() >= ((Wait) currentWaypoint).waitTime) {//Waited for the set time
                    resetTimer = false;
                    if (waypointIndex < path.size() - 1) {//Move on if not at the end of the path
                        waypointIndex++;
                    } else {//If we are waiting last then done... this shouldn't ever be reached.
                        break;
                    }
                }
                //TODO: Maybe do this... hold position during Wait
//                advancedRunToPoint(currentWaypoint, PIDActivationDistance, accuratePathFollowing, opMode, false, telemetry);
                drive.setMotorPowers(0,0,0,currentWaypoint.speed);//Should stop extraneous motion
                continue;//Keep repeating loop until time is up.
            }
            if (waypointIndex < path.size() - 1 && Math.abs(asPoint(robot).distance(asPoint(currentWaypoint))) < PIDActivationDistance) {
                //Past target... sort of. Get there and go to next one.
                if (!(currentWaypoint instanceof StopWaypoint) && currentWaypoint instanceof HeadingControlledWaypoint && ((HeadingControlledWaypoint) currentWaypoint).isStopping) {
                    advancedRunToPoint(currentWaypoint, PIDActivationDistance, opMode, true);
                    waypointIndex++;
                    continue;
                } else if (!(currentWaypoint instanceof StopWaypoint) && currentWaypoint instanceof HeadingControlledWaypoint && !((HeadingControlledWaypoint) currentWaypoint).isStopping) {
                    simpleRunToPoint(currentWaypoint, PIDActivationDistance);
                    if (Math.abs(asPoint(robot).distance(asPoint(currentWaypoint))) < currentWaypoint.minAccuracy) {
                        waypointIndex++;
                    }
                    continue;
                } else if (!(currentWaypoint instanceof HeadingControlledWaypoint)) {// Is a normal waypoint... rarely will get used... it might not even work anymore
                    waypointIndex++;
                    continue;
                }
            }

            //What type of waypoint is needed?
            Waypoint target;
            if (currentWaypoint instanceof StopWaypoint) {
                LineSegment line = new LineSegment(asPoint(robot), currentWaypoint);
                //As the robot's position is the first point in the line segment, there will only ever be one or zero intersections.
                ArrayList<Point> intersections = line.intersect(asPoint(robot),PIDActivationDistance);
                Point p;
                if (intersections.size() > 0) {
                    p = intersections.get(0);
                    target = new HeadingControlledWaypoint(p.getX(), p.getY(),((StopWaypoint) currentWaypoint).getTargetHeading(), currentWaypoint.speed, false,PIDActivationDistance);
                } else {
                    target = currentWaypoint;
                }
            } else {
                //Don't need to stop so accuracy is not necessary
                target = currentWaypoint;
            }

            if (target instanceof StopWaypoint) {
                advancedRunToPoint(target, PIDActivationDistance, opMode,true);
                break;//StopWaypoint should be the last in the path
            } else {
                simpleRunToPoint(target, PIDActivationDistance);
            }
        }
    }


    public Point asPoint(Pose pose) {
        return new Point(pose.getX(), pose.getY());
    }
    public Point asPoint(Waypoint waypoint) {
        return new Point(waypoint.getX(), waypoint.getY());
    }
}
