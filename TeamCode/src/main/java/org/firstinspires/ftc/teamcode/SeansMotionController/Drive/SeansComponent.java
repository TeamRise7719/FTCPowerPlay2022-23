package org.firstinspires.ftc.teamcode.SeansMotionController.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Encoder;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

/**
 * Created by Sean Cardosi on 12/20/22.
 */
public class SeansComponent {

    public ExpansionHubMotor leftLift;
    public ExpansionHubMotor rightLift;
    public Servo leftArm;
    public Servo rightArm;
    final static double COUNTS_PER_REV = 8192;//Encoder Counts
    final static double SPOOL_RADIUS = 1.884;//cm
    public Servo leftGrabber;
    public Servo rightGrabber;
    public Encoder liftEncoder;
    SeansSynchronousPID liftPID1;
    double LIFT_P1 = 0.055;
    double LIFT_I1 = 0.0;
    double LIFT_D1 = 0.0;
    SeansSynchronousPID liftPID2;
    double LIFT_P2 = 0.001;//0.001 0.055
    double LIFT_I2 = 0.0;
    double LIFT_D2 = 0.0;
    ExpansionHubEx hub;

    public SeansComponent(HardwareMap hardwareMap) {

        hub = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

        leftLift = (ExpansionHubMotor) hardwareMap.dcMotor.get("perpEncoder");
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift = (ExpansionHubMotor) hardwareMap.dcMotor.get("liftR");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm = hardwareMap.servo.get("left");
        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm = hardwareMap.servo.get("right");
        rightArm.setDirection(Servo.Direction.FORWARD);
        liftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "liftR"));
        liftEncoder.setDirection(Encoder.Direction.FORWARD);


//        stringPotentiometer = hardwareMap.analogInput.get("stringEnc");

        leftGrabber = hardwareMap.servo.get("clawL");
        leftGrabber.setDirection(Servo.Direction.REVERSE);
        rightGrabber = hardwareMap.servo.get("clawR");
        rightGrabber.setDirection(Servo.Direction.FORWARD);

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
        leftGrabber.setPosition(a);
        rightGrabber.setPosition(a);
    }

    public void moveArm(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    public boolean liftTo(double cm) {
        double curr = liftEncoder.getCurrentPosition();
        curr = encoderTicksToCentimeters(curr);
        double target = cm;
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
        double curr = liftEncoder.getCurrentPosition();
        curr = encoderTicksToCentimeters(curr);
        double target = cm;
        double error = target - curr;
        double liftPower = liftPID2.calculateUseError(error);
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);

        return Math.abs(target - curr) < 2;
    }

//    public double getStringPosition() {
//        return stringPotentiometer.getVoltage();
//    }

    public void getTelemetry(Telemetry telemetry) {
//        telemetry.addData("StringPosition",getStringPosition());
        telemetry.addData("Left Lift", leftLift.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("Right Lift", leftLift.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
    }

    public static double encoderTicksToCentimeters(double ticks) {
        return SPOOL_RADIUS * 2.0 * Math.PI * ticks / COUNTS_PER_REV;
    }

}
