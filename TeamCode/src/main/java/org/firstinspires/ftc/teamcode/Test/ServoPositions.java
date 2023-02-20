package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

/**
 * Created by Sean Cardosi on 1/11/23.
 */
@TeleOp
public class ServoPositions extends OpMode {

    SeansComponent c;
    double pos = 0.6375;
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
            c.odoServo.setPosition(pos);
        } else if (gamepad1.b && !bState) {
            pos -= 0.01;
            c.odoServo.setPosition(pos);
        }
        aState = gamepad1.a;
        bState = gamepad1.b;

        if (gamepad1.x) {
            c.odoServo.setPosition(0.6375);
        }
        if (gamepad1.y) {
            c.odoServo.setPosition(0.43);
        }

        pos = c.odoServo.getPosition();
        telemetry.addData("Position",pos);
//        telemetry.addData("Arm",c.rightArm.getPosition());
    }
}
