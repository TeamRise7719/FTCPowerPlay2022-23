package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftDistance {

    DcMotor lift;



    public LiftDistance(HardwareMap hardwareMap){
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(){

    }


    public void liftD(double distance){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double spoolC = 1.14;
        double COUNTS_PER_MOTOR_REV = 383.6;
        double InchesT = COUNTS_PER_MOTOR_REV / (spoolC*Math.PI);
        int move = ((int)(distance * InchesT));
        int newTarget = (lift.getCurrentPosition()+move);
        lift.setTargetPosition(newTarget);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(.4);
        while(lift.isBusy()){}
        lift.setPower(.3);
    }
}
