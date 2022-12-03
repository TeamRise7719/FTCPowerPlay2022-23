package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Test.LiftDistance;

@Autonomous(name = "Lift Raised", group = "tests")
public class LiftAuto extends LinearOpMode {

    LiftDistance liftl;
    @Override
    public void runOpMode() throws InterruptedException {
        liftl = new LiftDistance(hardwareMap);
        liftl.liftD(10);
    }
}
