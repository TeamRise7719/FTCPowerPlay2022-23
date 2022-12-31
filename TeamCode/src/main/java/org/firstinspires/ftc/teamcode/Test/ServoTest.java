package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Tests")
public class ServoTest extends OpMode {

    Servo right, left;

    double posit = 0.47;


    static final double INCREMENT   = 0.01;

    @Override
    public void init() {

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");

        right.setDirection(Servo.Direction.FORWARD);
        left.setDirection(Servo.Direction.REVERSE);

        right.setPosition(posit);
        left.setPosition(posit);

    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            posit = 0.47;
        }
        if (gamepad1.right_bumper) {
            posit = 0.597;
        }
        if (gamepad1.left_bumper) {
            posit = 0.37;
        }
        if (gamepad1.b) {
            posit = 0.47 + (gamepad1.left_trigger - gamepad1.right_trigger) / 4;
        }
        right.setPosition(posit);
        left.setPosition(posit);

        telemetry.addData("Position:", posit);
    }
}
