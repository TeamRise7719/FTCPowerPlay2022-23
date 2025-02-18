package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.PIDController;

/**
 * Created by Sean Cardosi on 2/7/23.
 */
@TeleOp
public class ArmMotorPIDTuning extends OpMode {

    double P = 1.4;//Start with a very low P and increase until fast with no bouncing
    //We don't need "I" for an arm
    double D = 0.0;//If arm is slow or can't really reach the target after tuning P, then increase P and add some D
    //If the arm can't hold its position after tuning, let me know and I'll make a PIDF controller
    AnalogInput pot;
    DcMotorEx armMotor;
    double targetVoltage = 2.26;//Change to the voltage the potentiometer reads when the arm is in the desired position.
    PIDController pid;

    @Override
    public void init() {

        pot = hardwareMap.analogInput.get("pot");
        armMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(new PIDCoefficients(P,0, D));

    }

    @Override
    public void loop() {
        double currentVoltage = pot.getVoltage();
        //If changing P does not seem to affect the voltage, change this to not be negative
        if (gamepad1.left_stick_y > 0.0 || gamepad1.left_stick_y < 0.0) {
            armMotor.setPower(gamepad1.left_stick_y);
        } else {
//            double power = pid.calculate(targetVoltage , currentVoltage);
            armMotor.setPower(0);
        }
//        double power = pid.calculate(targetVoltage , currentVoltage);
//        armMotor.setPower(power);
        telemetry.addData("Target Voltage",targetVoltage);
        telemetry.addData("Current Voltage",currentVoltage);
        telemetry.addData("Max Voltage",pot.getMaxVoltage());
        telemetry.addData("Error",targetVoltage - currentVoltage);
    }
}
