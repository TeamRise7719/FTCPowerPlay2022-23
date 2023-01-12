package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 9/3/22.
 */
public class Angle {

    /**
     * Ensures the given angle is within -2π to 2π.
     */
    public static double angleWrap(double angle) {
        double value = angle % (2 * Math.PI);
        if (Math.abs(value) > Math.PI) {
            value -= Math.copySign(2 * Math.PI, value);
        }
        return value;
    }
}
