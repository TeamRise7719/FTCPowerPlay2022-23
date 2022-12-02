package org.firstinspires.ftc.teamcode.Test.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Threaded Auto", group = "tests")
public class AutoTest extends LinearOpMode {
    ThreadCommands2 test2;

    @Override
    public void runOpMode() throws InterruptedException {
        test2.run();
    }

}
