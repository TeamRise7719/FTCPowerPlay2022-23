package org.firstinspires.ftc.teamcode.Vision.Tests.TreadingTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ThreadCommands3 extends Thread {

    public DcMotor lift;

    public ThreadCommands3 (HardwareMap hardwareMap){

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void run(){
        lift.setPower(-.4);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
