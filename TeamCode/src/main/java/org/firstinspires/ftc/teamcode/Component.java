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
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftGrabber = hardwareMap.servo.get("lGrab");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber = hardwareMap.servo.get("rGrab");
        rightGrabber.setDirection(Servo.Direction.REVERSE);
    }

    public void moveLift(double power) {
        lift.setPower(power);
    }

    public void stopLift() {
        lift.setPower(0.0);
    }

    public void grab(){
        leftGrabber.setPosition(60);
        rightGrabber.setPosition(60);
    }

    public void release() {
        leftGrabber.setPosition(0);
        rightGrabber.setPosition(0);
    }

    public void moveArm(double power) {
        arm.setPower(power);
    }

    public void stopArm() {
        arm.setPower(0);
    }

    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Lift Encoder", lift.getCurrentPosition());
        telemetry.addData("Arm Encoder", arm.getCurrentPosition());
        telemetry.addData("Left Grabber Position", leftGrabber.getPosition());
        telemetry.addData("Right Grabber Position", rightGrabber.getPosition());
    }

}
