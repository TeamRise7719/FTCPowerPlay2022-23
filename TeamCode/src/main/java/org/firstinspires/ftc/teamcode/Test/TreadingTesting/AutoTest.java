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
    Component component;
    SeansEncLibrary enc;
    LiftDistance lift;
    ThreadCommands test;
    ThreadCommands2 test2;

    @Override
    public void runOpMode() throws InterruptedException {
        component = new Component(hardwareMap);
        lift = new LiftDistance(hardwareMap);
        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        test = new ThreadCommands(hardwareMap);
        test2 = new ThreadCommands2(hardwareMap, telemetry, this);
        Thread myThread = new Thread(test);
        Thread myThread2 = new Thread(test2);
        enc.init();
        lift.init();
        component.init();
        test2.init();
        test.init();
        Runnable liftAction = () -> {component.liftD(5);};
        Thread liftThread = new Thread(liftAction);
        Runnable drive10 = () -> {enc.steeringDrive(10,false,false);};
        Thread driving = new Thread(drive10);

        waitForStart();

        component.moveLift(.1);
        sleep(1000);
        component.stopLift();
        lift.liftD(4);
        liftThread.start();
        driving.start();
        //myThread2.start();
        //myThread.start();
        while(myThread.isAlive()){}
        component.stopLift();
        sleep(5000);
        
    }

}
