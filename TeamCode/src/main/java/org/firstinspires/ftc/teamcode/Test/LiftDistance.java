package org.firstinspires.ftc.teamcode.Test;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftDistance {

    DcMotor lift;

    public LiftDistance(HardwareMap hardwareMap){
        lift = hardwareMap.dcMotor.get("lift");
    }

    public void liftD(double distance){

    }

}
