package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

/**
 * Created by Sean Cardosi on 2/15/23.
 */
@TeleOp
public class ArmSimpleMove extends OpMode {

    double P = 1.0;//Start with a very low P and increase until fast with no bouncing
    //We don't need "I" for an arm
    double D = 0.0;//If arm is slow or can't really reach the target after tuning P, then increase P and add some D
    //If the arm can't hold its position after tuning, let me know and I'll make a PIDF controller
    AnalogInput pot;
    DcMotorEx armMotor;
    SeansSynchronousPID pid;
    double targetVoltage = 1.284;
    boolean clawsOpen = false;
    boolean leftBumperState = true;
    private SeansComponent component;

    @Override
    public void init() {

        pot = hardwareMap.analogInput.get("pot");
        armMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new SeansSynchronousPID(P,0.0,D);
        pid.setOutputRange(-1.0,1.0);
        component = new SeansComponent(hardwareMap);
        component.init();
    }

    @Override
    public void start() {
        super.start();
        component.leftGrabber.setPosition(GlobalVariables.closeL);
        component.rightGrabber.setPosition(GlobalVariables.closeR);
    }


    @Override
    public void loop() {
        double currentVoltage = pot.getVoltage();
        //If changing P does not seem to affect the voltage, change this to not be negative
        if (gamepad1.dpad_left) {
            targetVoltage = 2.358;
        } else if (gamepad1.dpad_up) {
            targetVoltage = 1.284;
        } else if (gamepad1.dpad_right) {
            targetVoltage = 0.62;
        }
        if (gamepad1.left_bumper && !clawsOpen && !leftBumperState) {
            component.rightGrabber.setPosition(GlobalVariables.openR);
            component.leftGrabber.setPosition(GlobalVariables.openL);
            clawsOpen = true;
        } else if (gamepad1.left_bumper && clawsOpen && !leftBumperState) {
            component.rightGrabber.setPosition(GlobalVariables.closeR);
            component.leftGrabber.setPosition(GlobalVariables.closeL);
            clawsOpen = false;
        }
        leftBumperState = gamepad1.left_bumper;

        double power = pid.calculateUseError(targetVoltage - currentVoltage);

        armMotor.setPower(power);
        telemetry.addData("Target Voltage",targetVoltage);
        telemetry.addData("Current Voltage",currentVoltage);
        telemetry.addData("Max Voltage",pot.getMaxVoltage());
        telemetry.addData("Error",targetVoltage - currentVoltage);
    }
}
