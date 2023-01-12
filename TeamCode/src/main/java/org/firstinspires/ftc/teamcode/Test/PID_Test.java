package org.firstinspires.ftc.teamcode.Test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SeansMotionController.Vision.Subsystems.Components.Component;


/**
 * Created by Jordan Nuthalapaty 10/26/2022
 */
@Disabled
@Autonomous(name = "Maverick Test Auto", group = "Maverick Auto")
public class PID_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Drivetrain Legion = new Drivetrain(hardwareMap);
        Component program = new Component(hardwareMap);


//        program.init();
//
//        waitForStart();
//        program.moveLift(1);
//        sleep(500);
//        program.stopLift();
//        program.moveArm(0);
//        sleep(1000);
//        program.moveLift(-1);
//        sleep(500);
//        program.stopLift();
    }
}
