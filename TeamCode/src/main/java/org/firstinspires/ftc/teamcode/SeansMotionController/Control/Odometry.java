package org.firstinspires.ftc.teamcode.SeansMotionController.Control;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Encoder;

import java.util.Arrays;
import java.util.List;

/**
 * Created by Sean Cardosi on 8/29/22.
 */
public class Odometry {

    private Encoder left;
    private Encoder right;
    private Encoder perp;

    /*final*/public static double TRACKWIDTH = 33.2;//34.65;//38.65;//Testbot
    final static double FORWARD_OFFSET = 4.278;//-4.2;//4.5;//Testbot
    final static double COUNTS_PER_REV = 8192;//Encoder Counts
    final static double WHEEL_RADIUS = 1.75;//cm
    //Multipliers are: Actual Distance / Measured Distance
    //repeat and average values
    final static double X_MULTIPLIER = 1.0;//1.010074294;
    final static double Y_MULTIPLIER = 1.0;//1.030195033;

    public Odometry(HardwareMap hardwareMap) {
        left = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
//        left.setDirection(Encoder.Direction.REVERSE);//Testbot
        left.setDirection(Encoder.Direction.FORWARD);
        right = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        right.setDirection(Encoder.Direction.REVERSE);//Compbot
        perp = new Encoder(hardwareMap.get(DcMotorEx.class, "perpEncoder"));
        perp.setDirection(Encoder.Direction.REVERSE);//Testbot and Compbot
    }

    public static double encoderTicksToCentimeters(double ticks) {
        return WHEEL_RADIUS * 2.0 * Math.PI * ticks / COUNTS_PER_REV;
    }

    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToCentimeters(left.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToCentimeters(right.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToCentimeters(perp.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToCentimeters(left.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToCentimeters(right.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToCentimeters(perp.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
