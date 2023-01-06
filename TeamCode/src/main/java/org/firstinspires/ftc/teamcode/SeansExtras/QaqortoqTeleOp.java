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
    boolean is90 = false;
    boolean is45 = false;
    boolean isFront = false;
    boolean isBack = false;
    boolean isUp = false;
    boolean rightBumper2state = false;
    boolean leftBumper2State = false;

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
        component.setClaw(positClaw);
        component.setArm(0.27);//Set arm to front 45 position
        is45 = true;
        isFront = true;
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
        if (gamepad2.dpad_up && !clawsOpen) {
            isUp = true;
            is45 = false;
            is90 = false;
            component.setArm(0.44);
        }

        if (gamepad2.right_bumper && !rightBumper2state) {
            if (!isUp) {
                if (is90) {//Swap to 45 position
                    is45 = true;
                    is90 = false;
                    if (isFront) {
                        component.setArm(0.27);
                    } else if (isBack) {
                        component.setArm(0.625);
                    }
                } else if (is45) {//Swap to 90 position
                    is45 = false;
                    is90 = true;
                    if (isFront) {
                        component.setArm(0.34);
                    } else if (isBack) {
                        component.setArm(0.55);
                    }
                }
            } else {//Go from up to 90 position
                isUp = false;
                is90 = true;
                is45 = false;
                if (isFront) {
                    component.setArm(0.34);
                } else if (isBack) {
                    component.setArm(0.55);
                }
            }
        }
        rightBumper2state = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !leftBumper2State && !clawsOpen) {
            if (isFront) {//Change everything to the back side
                isFront = false;
                isBack = true;
                if (is90) {
                    component.setArm(0.55);
                } else if (is45) {
                    component.setArm(0.625);
                }
            } else if (isBack) {//Change everything to the front side
                isFront = true;
                isBack = false;
                if (is90) {
                    component.setArm(0.34);
                } else if (is45) {
                    component.setArm(0.27);
                }
            }
        }
        leftBumper2State = gamepad2.left_bumper;
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
    }
}
