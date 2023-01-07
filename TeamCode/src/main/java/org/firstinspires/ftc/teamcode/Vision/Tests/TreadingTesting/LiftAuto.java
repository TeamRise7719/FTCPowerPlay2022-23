package org.firstinspires.ftc.teamcode.Vision.Tests.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.Subsystems.Components.Component;
import org.firstinspires.ftc.teamcode.Vision.Subsystems.Components.LiftDistance;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansEncLibrary;

@Disabled
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

//        Runnable liftAction = () -> {component.stopLift(); sleep(3000);};
//        Thread liftThread = new Thread(liftAction);

        waitForStart();
        liftl.liftD(10);
        liftl.liftD(-10);
//        component.moveLift(.2);
//        sleep(2000);
//        liftl.liftD(4);
//        liftThread.start();
        //enc.steeringDrive(-40,false,true);
        //enc.arcTurn(180);
        //while (liftThread.isAlive()){}
    }
}
