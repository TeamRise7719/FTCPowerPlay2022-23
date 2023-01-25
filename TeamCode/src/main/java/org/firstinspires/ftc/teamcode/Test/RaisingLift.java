package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

@TeleOp (name = "Raising Lift", group = "Test")
public class RaisingLift extends OpMode {

//    DcMotor leftLift;
//    DcMotor rightLift;
    SeansComponent component;

    @Override
    public void init() {
        component = new SeansComponent(hardwareMap);
        component.init();
    }

    @Override
    public void loop() {
//        leftLift = hardwareMap.dcMotor.get("perpEncoder");
//        rightLift = hardwareMap.dcMotor.get("liftR");
//        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        if (gamepad1.y){
            component.leftLift.setPower(1);
            component.rightLift.setPower(1);
        } else if (gamepad1.a){
            component.leftLift.setPower(-1);
            component.rightLift.setPower(-1);
        } else {
            component.leftLift.setPower(0);
            component.rightLift.setPower(0);
        }
    }
}
