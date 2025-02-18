package org.firstinspires.ftc.teamcode.SeansMotionController.Drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.Localizer;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Pose;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Created by Sean Cardosi on 8/28/22.
 */
public class SeanDrivetrain {

    private final DcMotorEx lf, lr, rf, rr;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;
    private Orientation angles;
    Localizer localizer;

    public SeanDrivetrain(final HardwareMap hardwareMap) {

        //configuring the components
        lr = hardwareMap.get(DcMotorEx.class,"backLeft");
        lf = hardwareMap.get(DcMotorEx.class,"frontLeft");
        lr.setDirection(DcMotorEx.Direction.REVERSE);
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rr = hardwareMap.get(DcMotorEx.class,"backRight");
        rf = hardwareMap.get(DcMotorEx.class,"frontRight");
        rr.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        localizer = new Localizer(hardwareMap, new Pose(0,0,0.0));

    }

    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    /**
     * Fetch all once-per-time-slice values.
     * <p>
     * Call this either in your OpMode::loop function or in your while(opModeIsActive())
     * loops in your autonomous. It refresh gyro and other values that are computationally
     * expensive.
     */
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);//ZYX
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _lf Left front motor
     * @param _lr Left rear motor
     * @param _rf Right front motor
     * @param _rr Right rear motor
     */
    private void setMotors(double _lf, double _lr, double _rf, double _rr) {
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);
        lf.setPower(_lf / scale);
        lr.setPower(_lr / scale);
        rf.setPower(_rf / scale);
        rr.setPower(_rr / scale);
    }

    public void drive(Gamepad gamepad1, Telemetry telemetry) {
        loop();
        localizer.updatePose();

        final double gx = gamepad1.left_stick_x;
        final double gy = -gamepad1.left_stick_y;
        final double r = (gamepad1.right_stick_x);
//        final double direction = Math.atan2(x, y) + getHeading();
        double heading = -getHeading();
        final double speed = Math.min(1.0, Math.sqrt(gx * gx + gy * gy));
        double x = gx * Math.cos(heading) - gy * Math.sin(heading);
        double y = gx * Math.sin(heading) + gy * Math.cos(heading);

//        double l = Math.max(Math.abs(gy) + Math.abs(gx) + Math.abs(r),1);
        double lf_ = (y + x + r) /*/ l*/;//speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        double lr_ = (y - x + r) /*/ l*/;//speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        double rf_ = (y - x - r) /*/ l*/;//speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        double rr_ = (y + x - r) /*/ l*/;//speed * Math.sin(direction + Math.PI / 4.0) - rotation;
        List<Double> speeds = Arrays.asList(lf_,lr_,rf_,rr_);
        double l = Math.max(Collections.max(speeds),-Collections.min(speeds));
        if (l > 1) {
            lf_ /= l;
            lr_ /= l;
            rf_ /= l;
            rr_ /= l;
        }

        if(gamepad1.right_bumper) {//If we are in a state where we want to quarter the drive speed then do so.
            lf.setPower(lf_ / 4.0);
            lr.setPower(lr_ / 4.0);
            rf.setPower(rf_ / 4.0);
            rr.setPower(rr_ / 4.0);
        } else {//If we are not in a state where we want to quarter the drive speed well... then don't.
            //Full Speed
            lf.setPower(lf_);
            lr.setPower(lr_);
            rf.setPower(rf_);
            rr.setPower(rr_);
        }
//
//        telemetry.addData("Speeds","LeftFront: %f \n RightFront: %f \n LeftBack: %f \n Right Back: %f", lf_, rf_, lr_, rr_);
//        telemetry.addData("RAW Gyro: ",getRawHeading());
//        telemetry.addData("HeadingRadians: ",getHeading());
//        telemetry.addData("HeadingDegrees: ",Math.toDegrees(getHeading()));
//        telemetry.addData("Offset: ",headingOffset);
//        telemetry.update();
    }

    /**
     * SPECIALIZED: NOT FOR NORMAL USE
     * @param x
     * @param y
     * @param r
     * @param scale
     */
    public void setMotorPowers(double x, double y, double r, double scale) {

        double lf_ = (x + y + r);
        double lr_ = (x - y + r);
        double rf_ = (x - y - r);
        double rr_ = (x + y - r);
        List<Double> speeds = Arrays.asList(lf_,lr_,rf_,rr_);
        double l = Math.max(Collections.max(speeds),-Collections.min(speeds));
        if (l > 1) {
            lf_ /= l;
            lr_ /= l;
            rf_ /= l;
            rr_ /= l;
        }
        lf_ *= scale;
        lr_ *= scale;
        rf_ *= scale;
        rr_ *= scale;
        lf.setPower(lf_);
        lr.setPower(lr_);
        rf.setPower(rf_);
        rr.setPower(rr_);
//        telemetry.addData("Speeds","LeftFront: %f \n RightFront: %f \n LeftBack: %f \n Right Back: %f", lf_, rf_, lr_, rr_);
    }
    public void stopMotors() {
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);

    }

    public double getMaxMotorPower() {
        List<Double> speeds = Arrays.asList(Math.abs(lf.getPower()),Math.abs(lr.getPower()),Math.abs(rf.getPower()),Math.abs(rr.getPower()));
        return Collections.max(speeds);
    }
}