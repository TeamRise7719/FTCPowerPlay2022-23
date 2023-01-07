package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftDistance {

    DcMotor lift;
    DcMotor liftR;



    public LiftDistance(HardwareMap hardwareMap){
        lift = hardwareMap.dcMotor.get("perpEncoder");
        liftR = hardwareMap.dcMotor.get("liftR");
        lift.setDirection(DcMotor.Direction.FORWARD);
        liftR.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(){

    }

    public void liftD(double distance){
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double spoolC = 1.40358268;
        double COUNTS_PER_MOTOR_REV = 28;
        double InchesT = COUNTS_PER_MOTOR_REV / (spoolC*Math.PI);
        int move = ((int)(distance * InchesT));
        int newTarget = (liftR.getCurrentPosition() + move);
        liftR.setTargetPosition(-900);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setPower(.6);
        while(liftR.isBusy()){ lift.setPower(.6);}
    }
}
