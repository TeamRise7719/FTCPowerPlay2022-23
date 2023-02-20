package org.firstinspires.ftc.teamcode.SeansMotionController.Drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Encoder;

/**
 * Created by Sean Cardosi on 12/20/22.
 */
public class SeansComponent {

    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
//    public DcMotor armMotor;
//    public AnalogInput pot;
    final static double COUNTS_PER_REV = 8192;//Encoder Counts
    final static double SPOOL_RADIUS = 1.884;//cm
    public Servo leftGrabber;
    public Servo rightGrabber;
    public Servo odoServo;
    public Encoder liftEncoder;
    SeansSynchronousPID liftPID1;
    double LIFT_P1 = 0.055;
    double LIFT_I1 = 0.0;
    double LIFT_D1 = 0.0;
    SeansSynchronousPID liftPID2;
    double LIFT_P2 = 0.001;//0.001 0.055
    double LIFT_I2 = 0.0;
    double LIFT_D2 = 0.0;

    public SeansComponent(HardwareMap hardwareMap) {


        leftLift = hardwareMap.get(DcMotorEx.class,"perpEncoder");
        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightLift = hardwareMap.get(DcMotorEx.class,"liftR");
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "liftR"));
        liftEncoder.setDirection(Encoder.Direction.FORWARD);

//        armMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
//        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


//        stringPotentiometer = hardwareMap.analogInput.get("stringEnc");

        leftGrabber = hardwareMap.servo.get("clawL");
        leftGrabber.setDirection(Servo.Direction.REVERSE);
        rightGrabber = hardwareMap.servo.get("clawR");
        rightGrabber.setDirection(Servo.Direction.FORWARD);

        odoServo = hardwareMap.servo.get("odoServo");
//        pot = hardwareMap.analogInput.get("pot");

        liftPID1 = new SeansSynchronousPID(LIFT_P1,LIFT_I1,LIFT_D1);
        liftPID1.setOutputRange(-1.0,1.0);
        liftPID2 = new SeansSynchronousPID(LIFT_P2,LIFT_I2,LIFT_D2);
        liftPID2.setOutputRange(-1.0,1.0);

    }

    public void init() {
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveLift(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void stopLift() {
        leftLift.setPower(0.01);
        rightLift.setPower(0.01);
    }

//    public void setArm(double power) {
//        armMotor.setPower(power);
//    }

    public void setClaw(double a) {
        leftGrabber.setPosition(a);
        rightGrabber.setPosition(a);
    }

    public boolean liftTo(double cm) {
        double curr = liftEncoder.getCurrentPosition();
        curr = encoderTicksToCentimeters(curr);
        double target = cm;
        double error = curr - target;
        double liftPower = liftPID1.calculateUseError(error);
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);


        return Math.abs(curr - target) < 2;
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

    public void open() {
        leftGrabber.setPosition(GlobalVariables.openL);
        rightGrabber.setPosition(GlobalVariables.openR);
    }
    public void close() {
        leftGrabber.setPosition(GlobalVariables.closeL);
        rightGrabber.setPosition(GlobalVariables.closeR);
    }

    public void setArm(double blah) {

    }

    public void getTelemetry(Telemetry telemetry) {
//        telemetry.addData("StringPosition",getStringPosition());
    }

    public void openM() {
        leftGrabber.setPosition(GlobalVariables.openLMega);
        rightGrabber.setPosition(GlobalVariables.openRMega);
    }

    public static double encoderTicksToCentimeters(double ticks) {
        return SPOOL_RADIUS * 2.0 * Math.PI * ticks / COUNTS_PER_REV;
    }

}
