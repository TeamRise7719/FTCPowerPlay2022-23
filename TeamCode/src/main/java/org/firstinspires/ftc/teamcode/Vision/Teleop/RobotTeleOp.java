package org.firstinspires.ftc.teamcode.Vision.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.Subsystems.Components.Component;
import org.firstinspires.ftc.teamcode.Vision.Subsystems.Driving.Drivetrain;

/**
 * Created by Sean Cardosi on 8/28/22.
 */
@Disabled
@TeleOp(name = "RobotTeleOp", group = "Octavius")
public class RobotTeleOp extends OpMode {

    private Drivetrain robot;
    private Component component;
    private boolean isReady = false;
    boolean isLiftHigh = false;
    boolean isLiftUp = false;
    boolean isLiftDown = true;
    boolean isGrabberClosed = true;
    final double MAX_LIFT_ENCODERS = 5000;
    final double MIN_LIFT_ENCODERS = 200;
    final double MIN_SWING_ENCODERS = 1000;

    @Override
    public void init() {

        //Initialize robot
        robot = new Drivetrain(hardwareMap);
        robot.runUsingEncoders();
        component = new Component(hardwareMap);
        isReady = true;
        isLiftHigh = false;
        isLiftUp = false;
        isLiftDown = true;
        isGrabberClosed = true;
        component.grab();
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
        if (gamepad2.left_bumper) {
            component.release();
        } else if (gamepad2.right_bumper) {
            component.grab();
        }
//        component.leftGrabber.setPosition(gamepad2.left_trigger);
//        component.rightGrabber.setPosition(gamepad2.right_trigger);
//        telemetry.addData("Right", gamepad2.right_trigger);
//        telemetry.addData("Left", gamepad2.left_trigger);
        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\


        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
        if (gamepad2.right_stick_y < -0.1) {
            component.moveLift(1.0);
        } else if (gamepad2.right_stick_y > 0.1) {
            component.moveLift(-0.1);
        } else {
            component.stopLift();
        }
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\


        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
        if (gamepad2.b /*&& isLiftHigh*/) {
            component.moveArm(1.0);
        } else if (gamepad2.x /*&& isLiftHigh*/) {
            component.moveArm(0.1);
        }
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
    }
}
