package org.firstinspires.ftc.teamcode.Qaqortoq.Teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Driving.QaqortoqDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.Localizer;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.PIDController;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Pose;

import java.util.List;

/**
 * Created by Sean Cardosi on 3/1/23.
 */

@TeleOp(name = "Connor TeleOp", group = "Connor")
public class ConnorTeleOp extends OpMode {

    private QaqortoqDrivetrain robot;
    private SeansComponent component;
    private boolean isReady = false;
    double lastHeight;
    boolean clawsOpen = false;
    boolean leftBumperState = true;
    boolean is90 = false;
    boolean is45 = false;
    boolean isFront = false;
    boolean isBack = false;
    boolean isUp = false;
    boolean onWayDown = false;
    boolean onWayUp = false;
    boolean rightBumper2State = false;
    boolean leftBumper2State = false;
    boolean rightTrigger2State = false;
    boolean leftTrigger2State = false;
    boolean dpadUp2State = false;
    double lastPosition;
    Localizer l;
    double armTarget = GlobalVariables.front45;
    PIDController armPID;
    List<LynxModule> allHubs;
    double armFFAngle = 135;
    double kcos = -0.1;
    double armP = 0.9;
    double armI = 0.0;
    double armD = 0.02;
    AnalogInput pot;
    DcMotorEx armMotor;
    PIDController goingUp;

    double upP = 1.7;
    double upI = 0.0;
    double upD = 0.0;

    @Override
    public void init() {

        //Initialize robot

        pot = hardwareMap.analogInput.get("pot");

        robot = new QaqortoqDrivetrain(hardwareMap);
        robot.runUsingEncoders();
        l = new Localizer(hardwareMap, new Pose(0,0,0));
        component = new SeansComponent(hardwareMap);
        isReady = true;
//        component.grab();
        component.init();
        armPID = new PIDController(new PIDCoefficients(armP,armI,armD));
        goingUp = new PIDController(new PIDCoefficients(upP,upI,upD));
        armMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void init_loop() {
        if (isReady) {
            telemetry.addData(">", "Robot Ready!");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        super.start();
        component.leftGrabber.setPosition(GlobalVariables.closeL);
        component.rightGrabber.setPosition(GlobalVariables.closeR);
//        component.setArm(GlobalVariables.front45);//Set arm to front 45 position
        lastPosition = GlobalVariables.back45;
        is45 = true;
        isFront = false;
        isBack = true;
        onWayUp = true;
        armTarget = GlobalVariables.back45;
        armFFAngle = 405;//180 is the front 90 position
        component.odoServo.setPosition(GlobalVariables.odoUp);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        l.updatePose();
        telemetry.addData("Pose","X: %f, Y: %f",l.getX(),l.getY());
        telemetry.addData("PoseHeading","Heading: %f", Math.toDegrees(l.getHeading()));
        component.getTelemetry(telemetry);


        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
        robot.drive(gamepad1, telemetry);

        if (gamepad1.x) {
            robot.resetHeading();
        }
        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\


        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
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

        telemetry.addData("ClawR", component.rightGrabber.getPosition());
        telemetry.addData("ClawL", component.leftGrabber.getPosition());
        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\



        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
        if (gamepad2.right_stick_y > 0.0) {
            component.leftLift.setPower(1.0);
            component.rightLift.setPower(1.0);
            lastHeight = component.rightLift.getCurrentPosition();
        } else if (gamepad2.right_stick_y < 0.0) {
            component.leftLift.setPower(-1.0);
            component.rightLift.setPower(-1.0);
            lastHeight = component.rightLift.getCurrentPosition();
        } else {
            double targetInCm = SeansComponent.encoderTicksToCentimeters(lastHeight);
            component.holdLift(targetInCm);
        }
        telemetry.addData("Lift Right", component.liftEncoder.getCurrentPosition());
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\



        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
        if (gamepad2.dpad_up && !clawsOpen) {
            lastPosition = armTarget;
            onWayUp = true;
            onWayDown = false;
            armTarget = GlobalVariables.up;
            armFFAngle = 270;
        }

        if (gamepad2.left_trigger > 0.0 && !leftTrigger2State) {//Front 90
            lastPosition = armTarget;
            armTarget = GlobalVariables.front90;
            armFFAngle = 180;
            if (lastPosition == GlobalVariables.front45) {
                onWayDown = false;
                onWayUp = true;
            } else {
                onWayDown = true;
                onWayUp = false;
            }
        }
        leftTrigger2State = gamepad2.left_trigger > 0.0;

        if (gamepad2.right_trigger > 0.0 && !rightTrigger2State) {//Back 90
            lastPosition = armTarget;
            armTarget = GlobalVariables.back90;
            armFFAngle = 360;
            if (lastPosition == GlobalVariables.back45) {
                onWayDown = false;
                onWayUp = true;
            } else {
                onWayDown = true;
                onWayUp = false;
            }
        }
        rightTrigger2State = gamepad2.right_trigger > 0.0;

        if (gamepad2.left_bumper && !leftBumper2State) {//Front 45
            lastPosition = armTarget;
            armTarget = GlobalVariables.front45;
            armFFAngle = 135;
            onWayDown = true;
            onWayUp = false;
        }
        leftBumper2State = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !rightBumper2State) {//Back 45
            lastPosition = armTarget;
            armTarget = GlobalVariables.back45;
            armFFAngle = 405;
            onWayDown = true;
            onWayUp = false;
        }
        rightBumper2State = gamepad2.right_bumper;


        double ffOutput = kcos * Math.cos((Math.toRadians(armFFAngle)));
        telemetry.addData("Target", armTarget);
        telemetry.addData("FFAngle", armFFAngle);
        telemetry.addData("Position", pot.getVoltage());
        telemetry.addData("Error", armTarget - pot.getVoltage());
        telemetry.addData("FF Output", ffOutput);
        double pidOutput;
        if (onWayUp) {
            pidOutput = goingUp.calculate(armTarget, pot.getVoltage());
        } else {
            pidOutput = armPID.calculate(armTarget, pot.getVoltage());
        }
        telemetry.addData("PID Output", pidOutput);
        telemetry.addData("Theoretical PIDF Output", pidOutput + ffOutput);
        telemetry.update();
        armMotor.setPower(pidOutput + ffOutput);
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
    }
}
