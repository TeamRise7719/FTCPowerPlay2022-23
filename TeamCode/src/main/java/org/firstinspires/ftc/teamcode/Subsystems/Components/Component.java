package org.firstinspires.ftc.teamcode.Subsystems.Components;

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
    public Servo arm;
    public Servo leftGrabber;
    public Servo rightGrabber;

    public Component(HardwareMap hardwareMap) {

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm = hardwareMap.servo.get("arm");
        arm.setDirection(Servo.Direction.FORWARD);

        leftGrabber = hardwareMap.servo.get("lGrab");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber = hardwareMap.servo.get("rGrab");
        rightGrabber.setDirection(Servo.Direction.REVERSE);

        moveArm(0.1);
    }

    public void init() {//Figure this out
        //Arm down when encoder is 150. Behind when greater than 150
    }

    public void moveLift(double power) {
        lift.setPower(power);
    }
    public void liftD(double distance){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double COUNTS_PER_MOTOR_REV = 383.6;
        double InchesT = COUNTS_PER_MOTOR_REV / Math.PI;
        int move = ((int)(distance * InchesT));
        int newTarget = (lift.getCurrentPosition()+move);
        lift.setTargetPosition(newTarget);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(.4);
        while(lift.isBusy()){}
        lift.setPower(.3);
    }

    public void stopLift() {
        lift.setPower(0.30);
    }

    public void grab(){
        leftGrabber.setPosition(0.38);//.5
        rightGrabber.setPosition(0.56);//1
    }

    public void release() {
        leftGrabber.setPosition(0.25);
        rightGrabber.setPosition(0.45);
    }

    public void moveArm(double position) {
        arm.setPosition(position);
    }

    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Lift Encoder", lift.getCurrentPosition());
        telemetry.addData("Arm Encoder", arm.getPosition());
        telemetry.addData("Left Grabber Position", leftGrabber.getPosition());
        telemetry.addData("Right Grabber Position", rightGrabber.getPosition());
    }

}
