package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Components.Component;
import org.firstinspires.ftc.teamcode.Subsystems.Sensing.SeansEncLibrary;


@Autonomous(name = "Lift Raised", group = "tests")
public class LiftAuto extends LinearOpMode {

    Component component;
    SeansEncLibrary enc;
    LiftDistance liftl;



    @Override
        public void runOpMode() throws InterruptedException {
        component = new Component(hardwareMap);
        liftl = new LiftDistance(hardwareMap);
        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        component.init();
        liftl.init();
        enc.init();

        Runnable liftAction = () -> {component.stopLift(); sleep(10000);};
        Thread liftThread = new Thread(liftAction);

        waitForStart();
        liftl.liftD(4);
        liftThread.start();
        enc.steeringDrive(-10,false,false);
        while (liftThread.isAlive()){}
    }
}
