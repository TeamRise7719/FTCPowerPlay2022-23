package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 9/3/22.
 */
public class Waypoint extends Point{

    public double minAccuracy;

    public Waypoint(double x, double y, double minAccuracy) {
        super(x,y);
        this.minAccuracy = minAccuracy;
    }

    public Waypoint(Point point, double minAccuracy) {
        super(point.getX(), point.getY());
        this.minAccuracy = minAccuracy;
    }

    public double getX() {
        return super.getX();
    }

    public double getY() {
        return super.getY();
    }
}
