package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Components.Component;
import org.firstinspires.ftc.teamcode.Subsystems.Sensing.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Test.LiftDistance;


@Autonomous(name = "Lift Raised", group = "tests")
public class LiftAuto extends LinearOpMode {

    LiftDistance liftl;
    SeansEncLibrary enc;



    @Override
        public void runOpMode() throws InterruptedException {
        liftl = new LiftDistance(hardwareMap);
        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        liftl.init();
        enc.init();
        waitForStart();
        liftl.liftD(10);
        sleep(3000);
        enc.steeringDrive(-10,false,false);


    }
}
