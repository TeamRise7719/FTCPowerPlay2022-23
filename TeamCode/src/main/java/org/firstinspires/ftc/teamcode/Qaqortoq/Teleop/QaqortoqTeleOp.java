package org.firstinspires.ftc.teamcode.Qaqortoq.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Driving.QaqortoqComponent;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Driving.QaqortoqDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;


/**
 * Created by Sean Cardosi on 8/28/22.
 */
@TeleOp(name = "TeleOp", group = "Octavius")
public class QaqortoqTeleOp extends OpMode {

    private QaqortoqDrivetrain robot;
    private SeansComponent component;
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
    double middle = 0.435;
    double backDown = 0.28;
    double backUp = 0.35;
    double frontDown = 0.59;
    double frontUp = 0.53;
    boolean is90 = false;
    boolean is45 = false;
    boolean isFront = false;
    boolean isBack = false;
    boolean isUp = false;
    boolean rightBumper2state = false;
    boolean leftBumper2State = false;
    boolean aButton2State = false;

    @Override
    public void init() {

        //Initialize robot
        robot = new QaqortoqDrivetrain(hardwareMap);
        robot.runUsingEncoders();

        component = new SeansComponent(hardwareMap);

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
        component.setClaw(0.68);
        component.setArm(0.465);//Set arm to front 45 position
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
//            component.release();
            component.setClaw(0.535);
            clawsOpen = true;
        } else if (gamepad1.left_bumper && clawsOpen && !leftBumperState) {
//            component.grab();
            component.setClaw(0.68);
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
            double targetInCm = lastHeight / 537.7 * (2 * Math.PI * 1.801);
            component.holdLift(targetInCm);
        }
//        telemetry.addData("Lift Left", component.leftLift.getCurrentPosition());
//        telemetry.addData("Lift Right", component.rightLift.getCurrentPosition());
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\


        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
        if (gamepad2.dpad_up && !clawsOpen) {
            isUp = true;
            is45 = false;
            is90 = false;
            component.setArm(0.55);
        }

        if (gamepad2.right_bumper && !rightBumper2state) {
            if (!isUp) {
                if (is90) {//Swap to 45 position
                    is45 = true;
                    is90 = false;
                    if (isFront) {
                        component.setArm(0.465);
                    } else if (isBack) {
                        component.setArm(0.63);
                    }
                } else if (is45) {//Swap to 90 position
                    is45 = false;
                    is90 = true;
                    if (isFront) {
                        component.setArm(0.495);
                    } else if (isBack) {
                        component.setArm(0.6);
                    }
                }
            } else {//Go from up to 90 position
                isUp = false;
                is90 = true;
                is45 = false;
                if (isFront) {
                    component.setArm(0.495);
                } else if (isBack) {
                    component.setArm(0.6);
                }
            }
        }
        rightBumper2state = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !leftBumper2State && !clawsOpen) {
            if (isFront) {//Change everything to the back side
                isFront = false;
                isBack = true;
                if (is90) {
                    component.setArm(0.6);
                } else if (is45) {
                    component.setArm(0.63);
                }
            } else if (isBack) {//Change everything to the front side
                isFront = true;
                isBack = false;
                if (is90) {
                    component.setArm(0.495);
                } else if (is45) {
                    component.setArm(0.465);
                }
            }
        }
        leftBumper2State = gamepad2.left_bumper;
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
    }
}
