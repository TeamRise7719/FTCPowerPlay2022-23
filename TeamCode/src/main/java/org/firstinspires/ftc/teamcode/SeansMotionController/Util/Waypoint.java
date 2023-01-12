package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 9/3/22.
 */
public class Waypoint extends Point{


    public Waypoint(double x, double y) {
        super(x,y);
    }
    public Waypoint(Point point) {
        super(point.getX(), point.getY());
    }

    public double getX() {
        return super.getX();
    }

    public double getY() {
        return super.getY();
    }
}
