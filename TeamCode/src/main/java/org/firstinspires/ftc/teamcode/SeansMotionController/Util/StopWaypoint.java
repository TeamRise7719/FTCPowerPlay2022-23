package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 9/5/22.
 */
public class StopWaypoint extends HeadingControlledWaypoint {

    public StopWaypoint(double x, double y, double targetHeading) {
        super(x, y, targetHeading,true,10);
    }

    public StopWaypoint(Point point, double targetHeading) {
        super(point, targetHeading,true,10);
    }

    public double getX() {
        return super.getX();
    }

    public double getY() {
        return super.getY();
    }

    public double getTargetHeading() {
        return super.getTargetHeading();
    }
}
