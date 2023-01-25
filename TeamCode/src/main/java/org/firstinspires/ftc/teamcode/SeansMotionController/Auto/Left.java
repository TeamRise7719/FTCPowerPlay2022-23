package org.firstinspires.ftc.teamcode.SeansMotionController.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.MotionController;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeanDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.ActionPoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Point;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.StopWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Wait;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Waypoint;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 1/13/23.
 */
@Autonomous(name = "Left", group = "Auto")
public class Left extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Tag1 = 1; // Tag ID 1 from the 36h11 family
    int Tag2 = 2; // Tag ID 1 from the 36h11 family
    int Tag3 = 3; // Tag ID 1 from the 36h11 family
    int detectedTag = 0;


    AprilTagDetection tagOfInterest = null;

    MotionController c;
    SeanDrivetrain d;
    SeansComponent component;

    @Override
    public void runOpMode() throws InterruptedException {
        c = new MotionController(hardwareMap);
        d = new SeanDrivetrain(hardwareMap);
        component = new SeansComponent(hardwareMap);
        component.init();
        d.resetHeading();

        Runnable lift1Thread = () -> {
            ElapsedTime e1 = new ElapsedTime();
            e1.reset();
            while (e1.seconds() < 6.5) {
                component.liftTo(62);
            }

            e1.reset();
            while (e1.seconds() < 2.5) {
                component.liftTo(55);
            }

            e1.reset();
            while (e1.seconds() < 2.2) {
                component.liftTo(13);
            }
            e1.reset();
            while (e1.seconds() < 4 && !this.isStopRequested()) {
                component.liftTo(64);
            }

            e1.reset();
            while (e1.seconds() < 2.5 && !this.isStopRequested()) {
                component.liftTo(52);
            }
        };


        Runnable arm1Thread = () -> {
            ElapsedTime e1 = new ElapsedTime();
            e1.reset();
            component.setArm(GlobalVariables.back90);
            while (e1.seconds() < 4) {}
//            component.setClaw(GlobalVariables.open);
            component.open();

            e1.reset();

            while (e1.seconds() < 0.8) {}
//            component.setClaw(GlobalVariables.closed);
            component.close();
            component.setArm(GlobalVariables.front45);
            e1.reset();

            while (e1.seconds() < 0.85) {}
//            component.setClaw(GlobalVariables.open);
            component.open();
            e1.reset();
            while (e1.seconds() < 2) {}
//            component.setClaw(GlobalVariables.closed);
            component.close();
            e1.reset();
            while (e1.seconds() < 1) {}
            component.setArm(GlobalVariables.back90);

            e1.reset();
            while (e1.seconds() < 4) {}
//            component.setClaw(GlobalVariables.open);
            component.open();
            e1.reset();
            while (e1.seconds() < 0.25) {}
//            component.setClaw(GlobalVariables.closed);
            component.close();
            component.setArm(GlobalVariables.front45);
        };

        waitForStart();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Side Camera"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//        while (!isStarted() && !isStopRequested()) {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections) {
//                    /*if(tag.id == 1) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }*/
//                    if (tag.id == Tag1 || tag.id == Tag2 || tag.id == Tag3) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound) {
//                    if (tagOfInterest.id == Tag1) {
//                        telemetry.addLine("Position 1");
//
//                    } else if (tagOfInterest.id == Tag2) {
//                        telemetry.addLine("Position 2");
//                    } else if (tagOfInterest.id == Tag3) {
//                        telemetry.addLine("Position 3");
//                    } else {
//                        telemetry.addLine("Something went wrong");
//                    }
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag:(");
//
//                    if(tagOfInterest == null) {
//                        telemetry.addLine("(A tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//            } else {
//                telemetry.addLine("Don't see tag:(");
//
//                if(tagOfInterest == null) {
//                    telemetry.addLine("(A tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//            telemetry.update();            sleep(20);
//        }
//
//        camera.closeCameraDevice();

//        component.setClaw(GlobalVariables.closed);//Close
        component.close();

        ArrayList<Waypoint> path = new ArrayList<>();
        ArrayList<ActionPoint> thingsToDo = new ArrayList<>();
        thingsToDo.add(new ActionPoint(new Point(-8,0),20,new Thread(lift1Thread)));
        thingsToDo.add(new ActionPoint(new Point(-8,-120),20,new Thread(arm1Thread)));


        path.add(new HeadingControlledWaypoint(-8, -20, Math.toRadians(0),true,1));
        path.add(new HeadingControlledWaypoint(-8, -140, Math.toRadians(180),false,3));
        path.add(new HeadingControlledWaypoint(2, -142, Math.toRadians(135),true,1));
        path.add(new Wait(3100));

        path.add(new HeadingControlledWaypoint(-3, -98, Math.toRadians(180),false,1));
        path.add(new HeadingControlledWaypoint(-66, -130, Math.toRadians(180),true,1));
        path.add(new Wait(1000));

        path.add(new HeadingControlledWaypoint(-8, -130, Math.toRadians(180),false,3));
        path.add(new HeadingControlledWaypoint(5, -142, Math.toRadians(135),true,1));
        path.add(new Wait(3000));

//        if (tagOfInterest.id == 1) {
//            path.add(new HeadingControlledWaypoint(-8, -93, Math.toRadians(180),false,1));
//            path.add(new StopWaypoint(-64, -130, Math.toRadians(180)));
//        } else if (tagOfInterest.id == 2) {
//            path.add(new StopWaypoint(-5, -130, Math.toRadians(180)));
//        } else if (tagOfInterest.id == 3) {
//            path.add(new HeadingControlledWaypoint(-8, -93, Math.toRadians(180),false,1));
//            path.add(new StopWaypoint(56, -130, Math.toRadians(180)));
//        } else {
//            path.add(new StopWaypoint(-5, -130, Math.toRadians(180)));
//        }

        path.add(new StopWaypoint(-5, -130, Math.toRadians(180)));

        c.followPath(path,thingsToDo,40,true,this, telemetry);
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
