package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 9/3/22.
 */
public class HeadingControlledWaypoint extends Waypoint {

    double targetHeading;
    public boolean isStopping;

    public HeadingControlledWaypoint(double x, double y,double targetHeading, double speed, boolean isStopping, double minAccuracy) {
        super(x, y,speed,minAccuracy);
        this.targetHeading = targetHeading;
        this.isStopping = isStopping;
    }
    public HeadingControlledWaypoint(Point point, double targetHeading, double speed, boolean isStopping, double minAccuracy) {
        super(point.getX(), point.getY(), speed, minAccuracy);
        this.targetHeading = targetHeading;
        this.isStopping = isStopping;
    }

    public double getX() {
        return super.getX();
    }

    public double getY() {
        return super.getY();
    }

    public double getTargetHeading() {
        return targetHeading;
    }
}
