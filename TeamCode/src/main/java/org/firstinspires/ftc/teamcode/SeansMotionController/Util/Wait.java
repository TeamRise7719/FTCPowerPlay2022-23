package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 1/6/23.
 */
public class Wait extends HeadingControlledWaypoint {

    public double waitTime;

    public Wait(double x, double y, double targetHeading, double milliseconds) {
        super(x, y, targetHeading);
        this.waitTime = milliseconds;
    }

    public Wait(Point point, double targetHeading, double milliseconds) {
        super(point, targetHeading);
        this.waitTime = milliseconds;
    }
}
