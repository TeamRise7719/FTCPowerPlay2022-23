package org.firstinspires.ftc.teamcode.SeansMotionController.Control;

import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Pose;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.SeansMotionController.Control.Odometry.FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.SeansMotionController.Control.Odometry.TRACKWIDTH;

/**
 * Created by Sean Cardosi on 8/30/22.
 */
public class Localizer {

    /**
     * +x is up
     * -x is down
     * +y is right
     * -y is left
     */

    private Pose currentPose;//Current robot pose
    private List<Double> lastWheelPositions;//dxl,dxr,dxh
    public double dxl = 0.0;
    public double dxr = 0.0;
    public double dxh = 0.0;
    public double dxc = 0.0;
    public double phi = 0.0;
    public double dxp = 0.0;
    final public double F = FORWARD_OFFSET;
    final public double L = TRACKWIDTH;
    public Odometry odometry;

    public Localizer(HardwareMap hardwareMap, Pose initialPose) {
        currentPose = initialPose;
        odometry = new Odometry(hardwareMap);
        lastWheelPositions = new ArrayList<>();
        lastWheelPositions.add(0.0);
        lastWheelPositions.add(0.0);
        lastWheelPositions.add(0.0);
    }

    public void updatePose() {
        List<Double> wheelPositions = odometry.getWheelPositions();
        dxl = wheelPositions.get(0) - lastWheelPositions.get(0);
        dxr = wheelPositions.get(1) - lastWheelPositions.get(1);
        dxh = wheelPositions.get(2) - lastWheelPositions.get(2);
        phi = (dxl - dxr) / L;
        dxc = (dxl + dxr) / 2.0;
        dxp = dxh - (F * phi);
        double h = currentPose.getHeading();
        /*
        To avoid a divide by 0 error, an incredibly small value is added to phi.
        Alternatively, one could use several Tailor Polynomials when phi is 0.
         */
        double p = phi + Double.MIN_VALUE;

        double dx = (dxc * ((cos(h) * sin(p) - sin(h) * (-cos(p) + 1)) / p)) + (dxp * ((cos(h) * (cos(p) - 1) - sin(h) * sin(p)) / p));
        double dy = (dxc * ((sin(h) * sin(p) + cos(h) * (-cos(p) + 1)) / p)) + (dxp * ((sin(h) * (cos(p) - 1) + cos(h) * sin(p)) / p));

        double newX = currentPose.getX() + dx;
        double newY = currentPose.getY() + dy;
        double newHeading = currentPose.getHeading() + phi;

        lastWheelPositions.set(0,wheelPositions.get(0));
        lastWheelPositions.set(1,wheelPositions.get(1));
        lastWheelPositions.set(2,wheelPositions.get(2));

        currentPose = new Pose(newX, newY, newHeading);
    }

    public Pose getPose() {
        return currentPose;
    }

    public double getX() {
        return currentPose.getX();
    }
    public double getY() {
        return currentPose.getY();
    }
    public double getHeading() {
        return currentPose.getHeading();
    }
}
