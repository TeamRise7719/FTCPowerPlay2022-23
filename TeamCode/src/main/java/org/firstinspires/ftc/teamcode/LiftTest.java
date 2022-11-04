package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sean Cardosi on 10/30/22.
 */
@Disabled
@TeleOp(name = "LiftTest", group = "Test")
public class LiftTest extends OpMode {

    DcMotor lift;

    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            lift.setPower(0.6);
        }
    }
}
