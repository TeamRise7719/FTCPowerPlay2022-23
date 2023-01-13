package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 11/1/22.
 */
public class ActionPoint extends Waypoint {

    Thread thread;
    double activationDistance;

    public ActionPoint(double x, double y, double activationDistance, Thread thread) {
        super(x, y,0);
        this.activationDistance = activationDistance;
        this.thread = thread;
    }

    public ActionPoint(Point point, double activationDistance, Thread thread) {
        super(point,0);
        this.activationDistance = activationDistance;
        this.thread = thread;
    }

    public void startThread() {
        thread.start();
    }

    public double getActivationDistance() {
        return activationDistance;
    }

    public String getAction() {
        return thread.getName();
    }

}

