package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ComponentV2 {

    DcMotor liftL, liftR;

    Servo right, left, clawR, clawL;

    public ComponentV2(HardwareMap hardwareMap) {
        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");
        clawR = hardwareMap.get(Servo.class, "clawR");
        clawL = hardwareMap.get(Servo.class, "clawL");

        liftL = hardwareMap.dcMotor.get("leftEncoder");
        liftL.setDirection(DcMotor.Direction.REVERSE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(Servo.Direction.FORWARD);
        left.setDirection(Servo.Direction.REVERSE);
        clawR.setDirection(Servo.Direction.FORWARD);
        clawL.setDirection(Servo.Direction.REVERSE);

        clawR.setPosition(0.45);
        clawL.setPosition(0.45);
    }

    public void init() {
        liftR.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void ArmFD() {
        right.setPosition(0.27);
        left.setPosition(0.27);
    }

    public void ArmF90() {
        right.setPosition(0.34);
        left.setPosition(0.34);
    }

    public void ArmM() {
        right.setPosition(0.44);
        left.setPosition(0.44);
    }

    public void ArmBD() {
        right.setPosition(0.625);
        left.setPosition(0.625);
    }

    public void ArmB90() {
        right.setPosition(0.55);
        left.setPosition(0.55);
    }

    public void grab(){
        clawR.setPosition(0.26);
        clawL.setPosition(0.26);
    }

    public void release(){
        clawR.setPosition(0.5);
        clawL.setPosition(0.5);
    }

}