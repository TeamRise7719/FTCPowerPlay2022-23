package org.firstinspires.ftc.teamcode.Test.TreadingTesting;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Jeremiah on 11/12/22.
 */
public class ThreadCommands extends Thread{

    public DcMotor lift;
    public ThreadCommands(HardwareMap hardwareMap){
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void run(){
        lift.setPower(.4);
    }




}
