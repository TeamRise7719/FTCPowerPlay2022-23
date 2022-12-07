package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Components.Component;

@Autonomous(name = "Speed", group = "tests")
public class TestingSpeedsAuto extends LinearOpMode {

    DcMotor rf,rb,lf,lb;


    public TestingSpeedsAuto(HardwareMap hardwareMap){
        rf = hardwareMap.dcMotor.get("frontRight");
        rb = hardwareMap.dcMotor.get("backRight");
        lf = hardwareMap.dcMotor.get("frontLeft");
        lb = hardwareMap.dcMotor.get("backLeft");

    }
    public void drive(){
        rf.setPower(-.4);
        rb.setPower(-.4);
        lb.setPower(-.4);
        lf.setPower(-.4);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Component component = new Component(hardwareMap);
        component.init();

        waitForStart();

        drive();
        sleep(5000);

    }
}
