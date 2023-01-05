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
    double positArm = 0.625;
    double positClaw = 0.45;
    double middle = 0.44;
    double backDown = 0.625;
    double backUp = 0.55;
    double frontDown = 0.27;
    double frontUp = 0.34;
    boolean bumperChangedL = false, bumperChangedR = false;
    boolean armIsFront = false, armIsUp = false;

    @Override
    public void init() {

        //Initialize robot
        robot = new QaqortoqDrivetrain(hardwareMap);
        robot.runUsingEncoders();

        component = new QaqortoqComponent(hardwareMap);

        isReady = true;
//        component.grab();
        component.init();
        component.setClaw(positClaw);
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
            component.setClaw(0.5);
            clawsOpen = true;
        } else if (gamepad1.left_bumper && clawsOpen && !leftBumperState) {
            component.setClaw(0.26);
            clawsOpen = false;
        }

        leftBumperState = gamepad1.left_bumper;
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
            double targetInCm = lastHeight / 384.5 * (2 * Math.PI * 1.801);
            component.holdLift(targetInCm);
        }
        telemetry.addData("Lift Left", component.leftLift.getCurrentPosition());
        telemetry.addData("Lift Right", component.rightLift.getCurrentPosition());
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\


        //----------------------------------------------=+(Arm)+=----------------------------------------------\\

        //Raises and lowers arm between 90 degree and down positions
        if (gamepad2.right_bumper && !bumperChangedR) {
            if (!armIsUp) {
                armIsUp = true;
            } else {
                armIsUp = false;
            }
            bumperChangedR = true;
        } else if (!gamepad2.right_bumper) {
            bumperChangedR = false;
        }
        // Flips arm
        if (gamepad2.left_bumper && !bumperChangedL && !clawsOpen) {
            if (!armIsFront) {
                armIsFront = true;
            } else {
                armIsFront = false;
            }
            bumperChangedL = true;
        } else if (!gamepad2.left_bumper) {
            bumperChangedL = false;
        }

        if (!clawsOpen && gamepad2.dpad_up) {
            positArm = middle;
        } else {
            if (armIsUp && armIsFront) {
                positArm = frontUp;
            }
            if (armIsUp && !armIsFront) {
                positArm = backUp;
            }
            if (!armIsUp && armIsFront) {
                positArm = frontDown;
            }
            if (!armIsUp && !armIsFront) {
                positArm = backDown;
            }
        }
        component.leftArm.setPosition(positArm);
        component.rightArm.setPosition(positArm);
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
    }
}
