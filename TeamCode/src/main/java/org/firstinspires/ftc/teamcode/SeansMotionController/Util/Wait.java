package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 1/6/23.
 */
public class Wait extends HeadingControlledWaypoint {

    public double waitTime;

    public Wait(double milliseconds) {
        super(0, 0, 0,false,0);
        this.waitTime = milliseconds;
    }
}
