package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 9/3/22.
 */
public class HeadingControlledWaypoint extends Waypoint {

    double targetHeading;

    public HeadingControlledWaypoint(double x, double y,double targetHeading) {
        super(x, y);
        this.targetHeading = targetHeading;
    }
    public HeadingControlledWaypoint(Point point, double targetHeading) {
        super(point.getX(), point.getY());
        this.targetHeading = targetHeading;
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
