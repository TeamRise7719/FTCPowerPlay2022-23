package org.firstinspires.ftc.teamcode.SeansMotionController.Vision.Tests.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Jeremiah on 11/12/22.
 */
@Disabled
@TeleOp(name = "threadTeleop", group = "Tests")
public class threadTeleop extends OpMode {

    ThreadCommands test;
    ThreadCommands2 test2;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.a){
            test2.run();
            test.run();
        }
    }
}
