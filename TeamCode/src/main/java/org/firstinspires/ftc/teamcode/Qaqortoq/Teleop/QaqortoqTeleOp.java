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
 * Created by Sean Cardosi on 8/28/22.
 */
@TeleOp(name = "TeleOp", group = "Octavius")
public class QaqortoqTeleOp extends OpMode {

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
    boolean is135 = false;
    boolean onWayDown = false;
    boolean onWayUp = false;
    boolean rightBumper2state = false;
    boolean leftBumper2State = false;
    double lastPosition;
    Localizer l;
    double armTarget = GlobalVariables.front45;
    PIDController armPID;
    List<LynxModule> allHubs;
    double armFFAngle = 135;
    double kcos = -0.1;
    double armP = 0.65;
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
        lastPosition = GlobalVariables.front45;
        is45 = true;
        isFront = true;
        onWayUp = true;
        armTarget = GlobalVariables.front45;
        armFFAngle = 135;//180 is the front 90 position
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
            double p = 1.0;
            if (gamepad2.right_trigger > 0.0) {
                p = 0.5;
            }
            component.leftLift.setPower(p);
            component.rightLift.setPower(p);
            lastHeight = component.rightLift.getCurrentPosition();
        } else if (gamepad2.right_stick_y < 0.0) {
            double p = -1.0;
            if (gamepad2.right_trigger > 0.0) {
                p = -0.5;
            }
            component.leftLift.setPower(p);
            component.rightLift.setPower(p);
            lastHeight = component.rightLift.getCurrentPosition();
        } else {
            double targetInCm = SeansComponent.encoderTicksToCentimeters(lastHeight);
            component.holdLift(targetInCm);
        }
        telemetry.addData("Lift Right", component.liftEncoder.getCurrentPosition());
        //----------------------------------------------=+(Lift)+=----------------------------------------------\\



        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
        if (gamepad2.dpad_up && !clawsOpen) {
            isUp = true;
//            component.setArm(GlobalVariables.up);
            armTarget = GlobalVariables.up;
        } else {
            isUp = false;
//            component.setArm(lastPosition);
            armTarget = lastPosition;

        }

        if (gamepad2.right_bumper && !rightBumper2state && !isUp) {
            if (onWayUp) {
                if (is45) {//Swap to 90 position
                    is45 = false;
                    is90 = true;
                    is135 = false;
                    if (isFront) {
//                        component.setArm(GlobalVariables.front90);
                        armTarget = GlobalVariables.front90;
                        lastPosition = GlobalVariables.front90;
                        armFFAngle = 180;
                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back90);
                        armTarget = GlobalVariables.back90;
                        lastPosition = GlobalVariables.back90;
                        armFFAngle = 0;
                    }
                } else if (is90) {//Swap to 135 position
                    is45 = false;
                    is90 = false;
                    is135 = true;
                    if (isFront) {
//                        component.setArm(GlobalVariables.front135);
                        armTarget = GlobalVariables.front135;
                        lastPosition = GlobalVariables.front135;
                        armFFAngle = 225;
                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back135);
                        armTarget = GlobalVariables.back135;
                        lastPosition = GlobalVariables.back135;
                        armFFAngle = 315;
                    }
                } else if (is135) {//Go down to 90
                    onWayDown = true;
                    onWayUp = false;
                    is45 = false;
                    is90 = true;
                    is135 = false;
                    if (isFront) {
//                        component.setArm(GlobalVariables.front90);
                        armTarget = GlobalVariables.front90;
                        lastPosition = GlobalVariables.front90;
                        armFFAngle = 180;
                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back90);
                        armTarget = GlobalVariables.back90;
                        lastPosition = GlobalVariables.back90;
                        armFFAngle = 360;
                    }
                }
            } else if (onWayDown) {
                if (is45) {//Swap to 90 position
                    is45 = false;
                    is90 = true;
                    is135 = false;
                    onWayUp = true;
                    onWayDown = false;
                    if (isFront) {
//                        component.setArm(GlobalVariables.front90);
                        armTarget = GlobalVariables.front90;
                        lastPosition = GlobalVariables.front90;
                        armFFAngle = 180;
                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back90);
                        armTarget = GlobalVariables.back90;
                        lastPosition = GlobalVariables.back90;
                        armFFAngle = 360;
                    }
                } else if (is90) {//Swap to 45 position
                    is45 = true;
                    is90 = false;
                    is135 = false;
                    if (isFront) {
//                        component.setArm(GlobalVariables.front45);
                        armTarget = GlobalVariables.front45;
                        lastPosition = GlobalVariables.front45;
                        armFFAngle = 135;
                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back45);
                        armTarget = GlobalVariables.back45;
                        lastPosition = GlobalVariables.back45;
                        armFFAngle = 405;
                    }
                } else if (is135) {//Go down to 90
                    is45 = false;
                    is90 = true;
                    is135 = false;
                    if (isFront) {
//                        component.setArm(GlobalVariables.front90);
                        armTarget = GlobalVariables.front90;
                        lastPosition = GlobalVariables.front90;
                        armFFAngle = 180;
                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back90);
                        armTarget = GlobalVariables.back90;
                        lastPosition = GlobalVariables.back90;
                        armFFAngle = 360;
                    }
                }
            }
//            lastPosition = component.rightArm.getPosition();
        }
        rightBumper2state = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !leftBumper2State && !clawsOpen) {
            if (isFront) {//Change everything to the back side
                isFront = false;
                isBack = true;
                if (is90) {
//                    component.setArm(GlobalVariables.back90);
                    armTarget = GlobalVariables.back90;
                    lastPosition = GlobalVariables.back90;
                    armFFAngle = 360;
                } else if (is45) {
//                    component.setArm(GlobalVariables.back45);
                    armTarget = GlobalVariables.back45;
                    lastPosition = GlobalVariables.back45;
                    armFFAngle = 405;
                } else if (is135) {
//                    component.setArm(GlobalVariables.back135);
                    armTarget = GlobalVariables.back135;
                    lastPosition = GlobalVariables.back135;
                    armFFAngle = 315;
                }
            } else if (isBack) {//Change everything to the front side
                isFront = true;
                isBack = false;
                if (is90) {
//                    component.setArm(GlobalVariables.front90);
                    armTarget = GlobalVariables.front90;
                    lastPosition = GlobalVariables.front90;
                    armFFAngle = 180;
                } else if (is45) {
//                    component.setArm(GlobalVariables.front45);
                    armTarget = GlobalVariables.front45;
                    lastPosition = GlobalVariables.front45;
                    armFFAngle = 135;
                } else if (is135) {
//                    component.setArm(GlobalVariables.front135);
                    armTarget = GlobalVariables.front135;
                    lastPosition = GlobalVariables.front135;
                    armFFAngle = 225;
                }
            }
            onWayUp = !onWayUp;
            onWayDown = !onWayDown;
//            lastPosition = component.rightArm.getPosition();
        }
        leftBumper2State = gamepad2.left_bumper;
        double ffOutput;
        if (armTarget != GlobalVariables.up) {
            ffOutput = kcos * Math.cos((Math.toRadians(armFFAngle)));
        } else {
            ffOutput = 0.0;
        }
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





//
//
//
//        if (gamepad2.dpad_up && !clawsOpen) {
////            isUp = true;
////            is45 = false;
////            is90 = false;
//            component.setArm(GlobalVariables.up);
//        } else {
//            component.setArm(lastPosition);
//        }
//
//        if (gamepad2.right_bumper && !rightBumper2state) {
//            if (!isUp) {
//                if (is90) {//Swap to 45 position
//                    is45 = true;
//                    is90 = false;
//                    if (isFront) {
//                        component.setArm(GlobalVariables.front45);
//                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back45);
//                    }
//                } else if (is45) {//Swap to 90 position
//                    is45 = false;
//                    is90 = true;
//                    if (isFront) {
//                        component.setArm(GlobalVariables.front90);
//                    } else if (isBack) {
//                        component.setArm(GlobalVariables.back90);
//                    }
//                }
//            } else {//Go from up to 90 position
//                isUp = false;
//                is90 = true;
//                is45 = false;
//                if (isFront) {
//                    component.setArm(GlobalVariables.front90);
//                } else if (isBack) {
//                    component.setArm(GlobalVariables.back90);
//                }
//            }
//            lastPosition = component.rightArm.getPosition();
//        }
//        rightBumper2state = gamepad2.right_bumper;
//
//        if (gamepad2.left_bumper && !leftBumper2State && !clawsOpen) {
//            if (isFront) {//Change everything to the back side
//                isFront = false;
//                isBack = true;
//                if (is90) {
//                    component.setArm(GlobalVariables.back90);
//                } else if (is45) {
//                    component.setArm(GlobalVariables.back45);
//                }
//            } else if (isBack) {//Change everything to the front side
//                isFront = true;
//                isBack = false;
//                if (is90) {
//                    component.setArm(GlobalVariables.front90);
//                } else if (is45) {
//                    component.setArm(GlobalVariables.front45);
//                }
//            }
//            lastPosition = component.rightArm.getPosition();
//        }
//        leftBumper2State = gamepad2.left_bumper;
        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
    }
}
