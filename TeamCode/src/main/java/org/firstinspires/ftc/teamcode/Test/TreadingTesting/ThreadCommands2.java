package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Sensing.SeansEncLibrary;


/**
 * Created by Jeremiah on 11/12/22.
 */
public class ThreadCommands2 implements Runnable{

    public Servo leftGrabber;
    public Servo rightGrabber;
    public DcMotor lift;
    SeansEncLibrary enc;

    public ThreadCommands2(HardwareMap hardwareMap){
        leftGrabber = hardwareMap.servo.get("lGrab");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber = hardwareMap.servo.get("rGrab");
        rightGrabber.setDirection(Servo.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        lift.setPower(.4);
        enc.steeringDrive(10,false,false);
    }

}