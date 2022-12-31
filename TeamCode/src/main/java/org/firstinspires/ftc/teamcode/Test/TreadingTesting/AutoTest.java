package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Components.Component;
import org.firstinspires.ftc.teamcode.Subsystems.Components.LiftDistance;
import org.firstinspires.ftc.teamcode.Subsystems.Sensing.SeansEncLibrary;


@Autonomous(name = "Threaded Auto", group = "tests")
public class AutoTest extends LinearOpMode {
    Component component;
    SeansEncLibrary enc;
    LiftDistance liftm;

    @Override
    public void runOpMode() throws InterruptedException {
        component = new Component(hardwareMap);
        liftm = new LiftDistance(hardwareMap);
        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();
        liftm.init();
        component.init();

        //Lambda functions
//        Runnable liftAction = () -> {liftm.liftD(10);};
//        Thread liftThread = new Thread(liftAction);
//        Runnable stopL = () -> {component.stopLift(); sleep(10000);};
//        Thread stopLi = new Thread(stopL);
//
//        waitForStart();
//
//        component.moveLift(.1);
//        sleep(2000);
//        component.stopLift();
//        liftm.liftD(4);
//        liftThread.start();
//        enc.steeringDrive(-10,false,false);
//        while(liftThread.isAlive()){}
//        stopLi.start();
//        while (stopLi.isAlive()){}
    }

}
