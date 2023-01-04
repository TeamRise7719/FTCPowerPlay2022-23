package org.firstinspires.ftc.teamcode.SeansExtras;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by Sean Cardosi on 8/28/22.
 */
@TeleOp(name = "TeleOp", group = "Octavius")
public class QaqortoqTeleOp extends OpMode {

    private QaqortoqDrivetrain robot;
    private QaqortoqComponent component;
    private boolean isReady = false;
    boolean isLiftHigh = false;
    boolean isLiftUp = false;
    boolean isLiftDown = true;
    boolean isGrabberClosed = true;
    final double MAX_LIFT_ENCODERS = 5000;
    final double MIN_LIFT_ENCODERS = 200;
    final double MIN_SWING_ENCODERS = 1000;
    double lastHeight;
    boolean clawsOpen = false;
    boolean leftBumperState = true;

    @Override
    public void init() {

        //Initialize robot
        robot = new QaqortoqDrivetrain(hardwareMap);
        robot.runUsingEncoders();

        component = new QaqortoqComponent(hardwareMap);

        isReady = true;
//        component.grab();
        component.init();
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
    }

    @Override
    public void loop() {
        component.getTelemetry(telemetry);


        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
        robot.drive(gamepad1, telemetry);

        if (gamepad1.x) {
            robot.resetHeading();
        }
        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\


        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
        if (gamepad1.left_bumper && !clawsOpen && !leftBumperState) {
            component.release();
            clawsOpen = true;
        } else if (gamepad1.left_bumper && clawsOpen && !leftBumperState) {
            component.grab();
            clawsOpen = false;
        }

        leftBumperState = gamepad1.left_bumper;
        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\


        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
        if (gamepad2.dpad_up) {
            component.leftLift.setPower(1.0);
            component.rightLift.setPower(1.0);
            lastHeight = component.rightLift.getCurrentPosition();
        } else if (gamepad2.dpad_down) {
            component.leftLift.setPower(-1.0);
            component.rightLift.setPower(-1.0);
            lastHeight = component.rightLift.getCurrentPosition();
        } else {
            double targetInCm = lastHeight / 384.5 * (2 * Math.PI * 1.801);
            component.holdLift(targetInCm);
        }
        telemetry.addData("Lift Left", component.leftLift.getCurrentPosition());
        telemetry.addData("Lift Right", component.rightLift.getCurrentPosition());
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\


        //----------------------------------------------=+(Arm)+=----------------------------------------------\\

        if (!clawsOpen) {
            if (gamepad2.a) {
                component.moveArm(0.47);
            }
            if (gamepad2.right_bumper) {
                component.moveArm(0.597);
            }
            if (gamepad2.left_bumper) {
                component.moveArm(0.37);
            }
        }
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
    }
}
