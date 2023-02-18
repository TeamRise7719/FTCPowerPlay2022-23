//package org.firstinspires.ftc.teamcode.Qaqortoq.Teleop;
//
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.GlobalVariables;
//import org.firstinspires.ftc.teamcode.OpenCV.PoleDetector;
//import org.firstinspires.ftc.teamcode.OpenCV.TeleOpPoleDetector;
//import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Driving.QaqortoqDrivetrain;
//import org.firstinspires.ftc.teamcode.SeansMotionController.Control.Localizer;
//import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;
//import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Pose;
//
//import java.util.List;
//
//
///**
// * Created by Sean Cardosi on 2/9/23.
// */
//
//@Disabled
//@TeleOp(name = "CameraTeleOp", group = "TeleOp")
//public class           CameraTeleOp extends OpMode {
//
//    private QaqortoqDrivetrain robot;
//    private SeansComponent component;
//    private boolean isReady = false;
//    boolean isLiftHigh = false;
//    boolean isLiftUp = false;
//    boolean isLiftDown = true;
//    boolean isGrabberClosed = true;
//    final double MAX_LIFT_ENCODERS = 5000;
//    final double MIN_LIFT_ENCODERS = 200;
//    final double MIN_SWING_ENCODERS = 1000;
//    double lastHeight;
//    boolean clawsOpen = false;
//    boolean leftBumperState = true;
//    boolean is90 = false;
//    boolean is45 = false;
//    boolean isFront = false;
//    boolean isBack = false;
//    boolean isUp = false;
//    boolean rightBumper2state = false;
//    boolean leftBumper2State = false;
//    boolean aButton2State = false;
//    double lastPosition;
//    boolean low = true;
//    boolean medium = true;
//    boolean high = true;
//    TeleOpPoleDetector d;
//    Localizer l;
//    boolean notified = false;
//    List<LynxModule> allHubs;
//
//    @Override
//    public void init() {
//
//        //Initialize robot
//        robot = new QaqortoqDrivetrain(hardwareMap);
//        robot.runUsingEncoders();
//        l = new Localizer(hardwareMap, new Pose(0, 0, 0));
//        d = new TeleOpPoleDetector(this, telemetry);
//        component = new SeansComponent(hardwareMap);
//        d.findPole();
//        isReady = true;
////        component.grab();
//        component.init();
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//    }
//
//    @Override
//    public void init_loop() {
//        if (isReady) {
//            telemetry.addData(">", "Robot Ready!");
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void start() {
//        super.start();
//        component.leftGrabber.setPosition(GlobalVariables.closeL);
//        component.rightGrabber.setPosition(GlobalVariables.closeR);
//        component.setArm(GlobalVariables.front45);//Set arm to front 45 position
//        lastPosition = GlobalVariables.front45;
//        is45 = true;
//        isFront = true;
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }
//        l.updatePose();
//        telemetry.addData("Pose", "X: %f, Y: %f", l.getX(), l.getY());
//        telemetry.addData("PoseHeading", "Heading: %f", Math.toDegrees(l.getHeading()));
//        component.getTelemetry(telemetry);
//
//
//        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
//        robot.drive(gamepad1, telemetry);
//
//        if (gamepad1.x) {
//            robot.resetHeading();
//        }
//        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
//
//
//        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
//        if (gamepad1.left_bumper && !clawsOpen && !leftBumperState) {
//            component.rightGrabber.setPosition(GlobalVariables.openR);
//            component.leftGrabber.setPosition(GlobalVariables.openL);
//            clawsOpen = true;
//        } else if (gamepad1.left_bumper && clawsOpen && !leftBumperState) {
//            component.rightGrabber.setPosition(GlobalVariables.closeR);
//            component.leftGrabber.setPosition(GlobalVariables.closeL);
//            clawsOpen = false;
//        }
//        leftBumperState = gamepad1.left_bumper;
//
//        telemetry.addData("ClawR", component.rightGrabber.getPosition());
//        telemetry.addData("ClawL", component.leftGrabber.getPosition());
//        //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
//
//        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
//        if (gamepad2.right_stick_y > 0.0) {
//            high = true;
//            medium = true;
//            low = true;
//            component.leftLift.setPower(1.0);
//            component.rightLift.setPower(1.0);
//            lastHeight = component.rightLift.getCurrentPosition();
//        } else if (gamepad2.right_stick_y < 0.0) {
//            high = true;
//            medium = true;
//            low = true;
//            component.leftLift.setPower(-1.0);
//            component.rightLift.setPower(-1.0);
//            lastHeight = component.rightLift.getCurrentPosition();
//        } else {
//            double targetInCm = SeansComponent.encoderTicksToCentimeters(lastHeight);
//            component.holdLift(targetInCm);
//        }
//        telemetry.addData("Lift Right", component.liftEncoder.getCurrentPosition());
//        //----------------------------------------------=+(Lift)+=----------------------------------------------\\
////        if(gamepad2.a){
////            //Low junction 36
////            low = false;
////            medium = true;
////            high = true;
////        }
////
////        if(!low){
////            low = component.liftTo(21.6);
////        }
////
////        if(gamepad2.b){
////            //Medium Junction 62
////            medium = false;
////            high = true;
////            low = true;
////        }
////        if(!medium){
////            medium = component.liftTo(47);
////        }
////        if(gamepad2.y){
////            //Tall Junction 87
////            high = false;
////            medium = true;
////            low = true;
////        }
////        if(!high){
////            high = component.liftTo(68);
////        }
//
//        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
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
//            notified = false;
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
//            notified = false;
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
//        //----------------------------------------------=+(Arm)+=----------------------------------------------\\
//
//
//        //----------------------------------------------=+(Pole Alignment Check)+=----------------------------------------------\\
//        if (!clawsOpen && !notified) {
//            if (isFront) {
//                if (is45) {
//                    if (Math.abs(GlobalVariables.forwardCameraTarget45 - d.frontForwardDistance()) < GlobalVariables.forwardCameraError45 && Math.abs(d.frontSideDistance()) < GlobalVariables.sideCameraError) {
//                        gamepad1.rumble(0.8, 0.8, 750);
//                        notified = true;
//                    }
//                } else if (is90 || isUp) {
//                    if (Math.abs(GlobalVariables.forwardCameraTarget90 - d.frontForwardDistance()) < GlobalVariables.forwardCameraError90 && Math.abs(d.frontSideDistance()) < GlobalVariables.sideCameraError) {
//                        gamepad1.rumble(0.8, 0.8, 750);
//                        notified = true;
//                    }
//                }
//            } else if (isBack) {
//                if (is45) {
//                    if (Math.abs(GlobalVariables.forwardCameraTarget45 - d.backForwardDistance()) < GlobalVariables.forwardCameraError45 && Math.abs(d.backSideDistance()) < GlobalVariables.sideCameraError) {
//                        gamepad1.rumble(0.8, 0.8, 750);
//                        notified = true;
//                    }
//                } else if (is90 || isUp) {
//                    if (Math.abs(GlobalVariables.forwardCameraTarget90 - d.backForwardDistance()) < GlobalVariables.forwardCameraError90 && Math.abs(d.backSideDistance()) < GlobalVariables.sideCameraError) {
//                        gamepad1.rumble(0.8, 0.8, 750);
//                        notified = true;
//                    }
//                }
//            }
//        }
//        if (clawsOpen) {
//            notified = false;
//        }
//        //----------------------------------------------=+(Pole Alignment Check)+=----------------------------------------------\\
//    }
//}
