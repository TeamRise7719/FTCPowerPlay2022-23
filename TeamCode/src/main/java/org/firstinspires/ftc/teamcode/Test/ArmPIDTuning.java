package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;

/**
 * Created by Sean Cardosi on 1/28/23.
 */

@TeleOp(name = "Arm PID Test", group = "Tests")
public class ArmPIDTuning extends OpMode {

    double P = 1;//Start with a very low P and increase until fast with no bouncing
    //We don't need "I" for an arm
    double D = 0.0;//If arm is slow or can't really reach the target after tuning P, then increase P and add some D
    //If the arm can't hold its position after tuning, let me know and I'll make a PIDF controller
    AnalogInput pot;
    CRServo leftArm;
    CRServo rightArm;
    double targetVoltage = 2.3;//Change to the voltage the potentiometer reads when the arm is in the desired position.
    SeansSynchronousPID pid;

    @Override
    public void init() {

        pot = hardwareMap.analogInput.get("pot");
        leftArm = hardwareMap.crservo.get("left");
        leftArm.setDirection(CRServo.Direction.REVERSE);
        rightArm = hardwareMap.crservo.get("right");
        rightArm.setDirection(CRServo.Direction.FORWARD);
        pid = new SeansSynchronousPID(P,0.0,D);
        pid.setOutputRange(-1.0,1.0);
        pid.setSetpoint(targetVoltage);
    }

    @Override
    public void loop() {
        double currentVoltage = pot.getVoltage();
        //If changing P does not seem to affect the voltage, change this to not be negative
        double power = -pid.calculate(currentVoltage);
        leftArm.setPower(power);
        rightArm.setPower(power);
        telemetry.addData("Target",targetVoltage);
        telemetry.addData("Current",currentVoltage);
        telemetry.addData("Error",targetVoltage - currentVoltage);
        telemetry.addData("LArm", leftArm.getPower());
        telemetry.update();

    }
}
