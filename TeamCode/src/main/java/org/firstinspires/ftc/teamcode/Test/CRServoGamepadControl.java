package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by Sean Cardosi on 1/30/23.
 */
@TeleOp
public class CRServoGamepadControl extends OpMode {

    CRServo leftArm;
    CRServo rightArm;

    @Override
    public void init() {
        leftArm = hardwareMap.crservo.get("left");
        leftArm.setDirection(CRServo.Direction.REVERSE);
        rightArm = hardwareMap.crservo.get("right");
        rightArm.setDirection(CRServo.Direction.FORWARD);
    }

    @Override
    public void loop() {

        double leftT = gamepad1.left_trigger;
        double rightT = gamepad1.right_trigger;

        if (leftT > 0.0) {
            leftArm.setPower(leftT);
            rightArm.setPower(leftT);
        } else if (rightT > 0.0) {
            leftArm.setPower(rightT);
            rightArm.setPower(rightT);
        } else {
            leftArm.setPower(0.0);
            rightArm.setPower(0.0);
        }

    }
}
