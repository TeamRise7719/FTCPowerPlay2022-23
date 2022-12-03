package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

/**
 * Created by Jeremiah on 11/12/22.
 */
public class ThreadCommands implements Runnable{

    DcMotor lift;
    public ThreadCommands(HardwareMap hardwareMap){
        lift = BlocksOpModeCompanion.hardwareMap.dcMotor.get("lift");
    }

    public void init(){
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftD(double distance){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double COUNTS_PER_MOTOR_REV = 383.6;
        double InchesT = COUNTS_PER_MOTOR_REV / 3.14159265359;
        int move = ((int)(distance * InchesT));
        int newTarget = (lift.getCurrentPosition()+move);
        lift.setTargetPosition(newTarget);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(.4);
        while(lift.isBusy()){}
        lift.setPower(.3);
    }


    @Override
    public void run(){
        liftD(10);
    }


}
