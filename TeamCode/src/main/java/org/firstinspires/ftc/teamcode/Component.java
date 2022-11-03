package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sean Cardosi on 10/28/22.
 */
public class Component {

    public DcMotor lift;
    public DcMotor arm;
    public Servo leftGrabber;
    public Servo rightGrabber;

    public Component(HardwareMap hardwareMap) {

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftGrabber = hardwareMap.servo.get("lGrab");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber = hardwareMap.servo.get("rGrab");
        rightGrabber.setDirection(Servo.Direction.REVERSE);
    }

    public void init() {//Figure this out
        //Arm down when encoder is 150. Behind when greater than 150
        arm.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void moveLift(double power) {
        lift.setPower(power);
    }

    public void stopLift() {
        lift.setPower(0.30);
    }

    public void grab(){
        leftGrabber.setPosition(0.55);//.5
        rightGrabber.setPosition(1.0);//1
    }

    public void release() {
        leftGrabber.setPosition(0.4);
        rightGrabber.setPosition(0.6);
    }

    public void moveArm(double power) {
        arm.setPower(power);
    }

    public void stopArm() {
        arm.setPower(0);
    }

    public double getArmEncoder() {
        return arm.getCurrentPosition();
    }
    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Lift Encoder", lift.getCurrentPosition());
        telemetry.addData("Arm Encoder", getArmEncoder());
        telemetry.addData("Left Grabber Position", leftGrabber.getPosition());
        telemetry.addData("Right Grabber Position", rightGrabber.getPosition());
    }

}
