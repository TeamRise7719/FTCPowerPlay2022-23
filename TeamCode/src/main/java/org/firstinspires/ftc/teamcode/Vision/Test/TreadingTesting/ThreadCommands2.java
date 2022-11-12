package org.firstinspires.ftc.teamcode.Vision.Test.TreadingTesting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jeremiah on 11/12/22.
 */
public class ThreadCommands2 extends Thread{

    public Servo leftGrabber;
    public Servo rightGrabber;

    public ThreadCommands2(HardwareMap hardwareMap){
        leftGrabber = hardwareMap.servo.get("lGrab");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber = hardwareMap.servo.get("rGrab");
        rightGrabber.setDirection(Servo.Direction.REVERSE);
    }

    public void run() {
//        leftGrabber.setPosition(0.4);
//        rightGrabber.setPosition(0.6);
        leftGrabber.setPosition(0.55);
        rightGrabber.setPosition(1.0);
    }
}
