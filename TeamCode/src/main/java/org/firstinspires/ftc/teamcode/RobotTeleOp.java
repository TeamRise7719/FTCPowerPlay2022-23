package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivetrain;

/**
 * Created by Sean Cardosi on 8/28/22.
 */
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
            component.grab();
        } else if (gamepad2.right_bumper) {
            component.release();
        }
        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\


        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
        if (gamepad2.right_stick_y > 0.1 /*&& !isLiftUp*/) {
            component.moveLift(0.8);
//            isLiftDown = false;
        } else if (gamepad2.right_stick_y < -0.1 /*&& !isLiftDown*/) {
            component.moveLift(-0.6);
//            isLiftUp = false;
//            isLiftHigh = false;
        } else {
            component.stopLift();
        }

//        if (component.lift.getCurrentPosition() >= MAX_LIFT_ENCODERS) {
//            isLiftUp = true;
//            isLiftHigh = true;
//            //TODO: FINISH THIS SO YOU DONT BREAK LIFT
//        } else if (component.lift.getCurrentPosition() <= MIN_LIFT_ENCODERS) {
//            isLiftDown = true;
//            isLiftHigh = false;
//        }

//        if (component.lift.getCurrentPosition() >= MIN_SWING_ENCODERS) {
//            isLiftHigh = true;
//        } else {
//            isLiftHigh = false;
//        }
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\


        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
        if (gamepad2.left_stick_y > 0.1 && isLiftHigh) {
            component.moveArm(0.6);
        } else if (gamepad2.left_stick_y < -0.1 && isLiftHigh) {
            component.moveArm(-0.6);
        } else {
            component.stopArm();
        }
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
//        if (gamepad2.a) {
//            telemetry.addData("Gamepad 2", "pressed a");
//            component.moveLift(0.6);
//        }
    }
}
