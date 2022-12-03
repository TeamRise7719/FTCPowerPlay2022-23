package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Components.Component;
import org.firstinspires.ftc.teamcode.Subsystems.Sensing.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Test.LiftDistance;


@Autonomous(name = "Threaded Auto", group = "tests")
public class AutoTest extends LinearOpMode {
    ThreadCommands2 test2;
    ThreadCommands test;
    Component component;
    LiftDistance lift;


    @Override
    public void runOpMode() throws InterruptedException {
        component = new Component(hardwareMap);
        test = new ThreadCommands(hardwareMap);
        test2 = new ThreadCommands2(hardwareMap,telemetry,this);
        lift = new LiftDistance(hardwareMap);
        lift.init();
        component.init();
        test2.init();

        waitForStart();

        component.moveLift(.1);
        sleep(1000);
        component.stopLift();
        lift.liftD(4);
        test.run();
        test2.run();


    }

}
