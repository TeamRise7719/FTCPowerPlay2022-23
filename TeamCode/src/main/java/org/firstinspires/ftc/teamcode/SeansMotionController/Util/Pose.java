package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 12/9/22.
 */
public class Pose extends Point {

    double heading;

    public Pose(Point point, double heading) {
        super(point.x, point.y);
        this.heading = heading;
    }

    public Pose(double x, double y, double heading) {
        super(x,y);
        this.heading = heading;
    }

    public double getHeading() {
        return heading;
    }

    public double getX() {
        return super.getX();
    }

    public double getY() {
        return super.getY();
    }
}
