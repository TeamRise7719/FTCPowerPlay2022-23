package org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Driving;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;

/**
 * Created by Sean Cardosi on 12/20/22.
 */
public class QaqortoqComponent {

    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo leftArm;
    public Servo rightArm;
    //    AnalogInput stringPotentiometer;
    public Servo grabber;
    SeansSynchronousPID liftPID1;
    double LIFT_P1 = 0.055;
    double LIFT_I1 = 0.0;
    double LIFT_D1 = 0.0;
    SeansSynchronousPID liftPID2;
    double LIFT_P2 = 0.001;
    double LIFT_I2 = 0.0;
    double LIFT_D2 = 0.0;

    public QaqortoqComponent(HardwareMap hardwareMap) {


        leftLift = hardwareMap.dcMotor.get("perpEncoder");
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift = hardwareMap.dcMotor.get("liftR");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm = hardwareMap.servo.get("left");
        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm = hardwareMap.servo.get("right");
        rightArm.setDirection(Servo.Direction.FORWARD);

//        stringPotentiometer = hardwareMap.analogInput.get("stringEnc");

        grabber = hardwareMap.servo.get("claw");
        grabber.setDirection(Servo.Direction.REVERSE);

        liftPID1 = new SeansSynchronousPID(LIFT_P1,LIFT_I1,LIFT_D1);
        liftPID1.setOutputRange(-1.0,1.0);
        liftPID2 = new SeansSynchronousPID(LIFT_P2,LIFT_I2,LIFT_D2);
        liftPID2.setOutputRange(-1.0,1.0);

    }

    public void init() {
        rightLift.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void moveLift(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void stopLift() {
        leftLift.setPower(0.01);
        rightLift.setPower(0.01);
    }

    public void setArm(double a) {
        leftArm.setPosition(a);
        rightArm.setPosition(a);
    }

    public void setClaw(double a) {
        grabber.setPosition(a);
    }

    public void moveArm(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    public boolean liftTo(double cm) {
        double curr = rightLift.getCurrentPosition();
        double COUNTS_PER_REV = 537.7;//Encoder Counts
        double target = cm * COUNTS_PER_REV / (2 * Math.PI * 1.801);
        double error = curr - target;
        double liftPower = liftPID1.calculateUseError(error);
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);

        if (Math.abs(curr - target) < 2) {
            return true;
        }
        return false;
    }

    public boolean holdLift(double cm) {
        double curr = rightLift.getCurrentPosition();
        double COUNTS_PER_REV = 537.7;//Encoder Counts
        double target = cm * COUNTS_PER_REV / (2 * Math.PI * 1.801);
        double error = curr - target;
        double liftPower = liftPID2.calculateUseError(error);
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);

        if (Math.abs(curr - target) < 2) {
            return true;
        }
        return false;
    }

//    public double getStringPosition() {
//        return stringPotentiometer.getVoltage();
//    }

    public void getTelemetry(Telemetry telemetry) {
//        telemetry.addData("StringPosition",getStringPosition());
    }

    //Quality of Life for Auto
    public void ArmFD() {
        rightArm.setPosition(0.27);
        leftArm.setPosition(0.27);
    }

    public void ArmF90() {
        rightArm.setPosition(0.34);
        leftArm.setPosition(0.34);
    }

    public void ArmM() {
        rightArm.setPosition(0.44);
        leftArm.setPosition(0.44);
    }

    public void ArmBD() {
        rightArm.setPosition(0.625);
        leftArm.setPosition(0.625);
    }

    public void ArmB90() {
        rightArm.setPosition(0.55);
        leftArm.setPosition(0.55);
    }

}
