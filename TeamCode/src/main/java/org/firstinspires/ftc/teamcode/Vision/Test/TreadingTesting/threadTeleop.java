package org.firstinspires.ftc.teamcode.Vision.Test.TreadingTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Jeremiah on 11/12/22.
 */
@TeleOp(name = "threadTeleop", group = "Tests")
public class threadTeleop extends OpMode {


    ThreadCommands test;
    ThreadCommands2 test2;


    @Override
    public void init() {
        test = new ThreadCommands(hardwareMap);
        test2 = new ThreadCommands2(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            test.start();
            test2.start();
        }
    }
}
