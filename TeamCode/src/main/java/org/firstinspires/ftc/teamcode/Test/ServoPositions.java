package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

/**
 * Created by Sean Cardosi on 1/11/23.
 */
@TeleOp
public class ServoPositions extends OpMode {

    SeansComponent c;
    double pos = 0.5;
    boolean aState = false;
    boolean bState = false;
    boolean rightBumpState = false;
    boolean tuningClaw = false;

    @Override
    public void init() {
        c = new SeansComponent(hardwareMap);
        c.init();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !aState) {
            pos += 0.01;
        } else if (gamepad1.b && !bState) {
            pos -= 0.01;
        }
        aState = gamepad1.a;
        bState = gamepad1.b;

        if (gamepad1.right_bumper && !rightBumpState) {
            pos = 0.5;
            tuningClaw = !tuningClaw;
        }
        rightBumpState = gamepad1.right_bumper;
        if (tuningClaw) {
            c.setClaw(pos);
        } else {
            c.setArm(pos);
        }
        telemetry.addData("Grabber",c.grabber.getPosition());
        telemetry.addData("Arm",c.rightArm.getPosition());
    }
}
