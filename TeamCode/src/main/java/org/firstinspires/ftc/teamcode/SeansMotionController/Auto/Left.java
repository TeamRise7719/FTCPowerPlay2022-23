package org.firstinspires.ftc.teamcode.SeansMotionController.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.MotionController;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeanDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.ArmState;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.LiftState;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.StateChange;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.StopWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Wait;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Waypoint;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 9/8/22.
 */
@Autonomous(name = "Left", group = "Auto")
public class Left extends LinearOpMode {

    MotionController c;
    SeanDrivetrain d;
    LiftState liftState;
    ArmState armState;
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

    @Override
    public void runOpMode() throws InterruptedException {
        c = new MotionController(hardwareMap);
        d = new SeanDrivetrain(hardwareMap);
        d.resetHeading();
        liftState = new LiftState("NOTHING",this,hardwareMap);
        armState = new ArmState("NOTHING",this,hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Side Camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    /*if(tag.id == 1) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }*/
                    if (tag.id == Tag1 || tag.id == Tag2 || tag.id == Tag3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    if (tagOfInterest.id == Tag1) {
                        telemetry.addLine("Position 1");

                    } else if (tagOfInterest.id == Tag2) {
                        telemetry.addLine("Position 2");
                    } else if (tagOfInterest.id == Tag3) {
                        telemetry.addLine("Position 3");
                    } else {
                        telemetry.addLine("Something went wrong");
                    }
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag:(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(A tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag:(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(A tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }

//        waitForStart();---------------------------


        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        liftState.start();
        armState.start();

        camera.closeCameraDevice();


        ArrayList<Waypoint> path = new ArrayList<>();

        path.add(new StateChange(armState,"CLOSE"));
        path.add(new StateChange(liftState,"RAISE40"));
        path.add(new HeadingControlledWaypoint(0, -10, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(-8, -20, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(-8, -50, Math.toRadians(0),1.0,false,8));
        path.add(new StateChange(armState,"FRONT135"));
        path.add(new HeadingControlledWaypoint(-8, -100, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(14, -142, Math.toRadians(-45),0.7,true,0));
        path.add(new StateChange(armState,"OPEN"));
//
        path.add(new Wait(200));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new StateChange(armState,"BACK45"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE11_5"));
        path.add(new HeadingControlledWaypoint(8, -128, Math.toRadians(0),1.0,false,8));
        path.add(new StateChange(armState,"OPENM"));
        path.add(new HeadingControlledWaypoint(-63, -144, Math.toRadians(0),1.0,true,0));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new Wait(200));
        path.add(new StateChange(armState,"FRONT135"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE42"));
        path.add(new HeadingControlledWaypoint(-52, -126, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(-15, -130, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(13.5, -141, Math.toRadians(-45),0.6,true,0));
        path.add(new StateChange(armState,"OPEN"));

        path.add(new Wait(200));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new StateChange(armState,"BACK45"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE8_5"));
        path.add(new HeadingControlledWaypoint(8, -125, Math.toRadians(0),1.0,false,8));
        path.add(new StateChange(armState,"OPENM"));
        path.add(new HeadingControlledWaypoint(-63, -140, Math.toRadians(0),1.0,true,0));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new Wait(200));
        path.add(new StateChange(armState,"FRONT135"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE42"));
        path.add(new HeadingControlledWaypoint(-52, -126, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(-15, -130, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(14.5, -141, Math.toRadians(-45),0.6,true,0));
        path.add(new StateChange(armState,"OPEN"));

        path.add(new Wait(200));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new StateChange(armState,"BACK45"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE6"));
        path.add(new HeadingControlledWaypoint(8, -120, Math.toRadians(0),1.0,false,8));
        path.add(new StateChange(armState,"OPENM"));
        path.add(new HeadingControlledWaypoint(-63, -136, Math.toRadians(0),1.0,true,0));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new Wait(200));
        path.add(new StateChange(armState,"FRONT135"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE42"));
        path.add(new HeadingControlledWaypoint(-52, -123, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(-15, -127, Math.toRadians(0),1.0,false,8));
        path.add(new HeadingControlledWaypoint(14.5, -140, Math.toRadians(-45),0.6,true,0));
        path.add(new StateChange(armState,"OPEN"));

        path.add(new Wait(200));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new StateChange(armState,"BACK45"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE3"));
        path.add(new HeadingControlledWaypoint(8, -120, Math.toRadians(0),1.0,false,8));
        path.add(new StateChange(armState,"OPENM"));
        path.add(new HeadingControlledWaypoint(-63, -133, Math.toRadians(0),1.0,true,0));//
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new Wait(200));
        path.add(new StateChange(armState,"FRONT135"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE42"));
        path.add(new HeadingControlledWaypoint(-52, -117, Math.toRadians(0),1.0,false,10));
        path.add(new HeadingControlledWaypoint(-8, -127, Math.toRadians(0),1.0,false,10));
        path.add(new HeadingControlledWaypoint(14.5, -139, Math.toRadians(-45),0.6,true,0));
        path.add(new StateChange(armState,"OPEN"));

        path.add(new Wait(200));
        path.add(new StateChange(armState,"CLOSE"));
        path.add(new StateChange(armState,"BACK45"));
        path.add(new Wait(200));
        path.add(new StateChange(liftState,"RAISE0"));
        path.add(new HeadingControlledWaypoint(0, -120, Math.toRadians(0),1.0,false,8));
        path.add(new StateChange(armState,"FRONT45"));

        if(tagOfInterest == null) {
            path.add(new HeadingControlledWaypoint(1, -120, Math.toRadians(0),1.0,true,0));
            path.add(new StopWaypoint(1, -100, Math.toRadians(-90),1.0));
        } else if (tagOfInterest.id == 1){
            path.add(new HeadingControlledWaypoint(-56, -120, Math.toRadians(-90),1.0,true,8));
            path.add(new StopWaypoint(-60,-100, Math.toRadians(-90),1.0));
        } else if (tagOfInterest.id == 2){
            path.add(new HeadingControlledWaypoint(1, -120, Math.toRadians(0),1.0,true,0));
            path.add(new StopWaypoint(1, -100, Math.toRadians(-90),1.0));
        } else if (tagOfInterest.id == 3){
            path.add(new HeadingControlledWaypoint(60, -120, Math.toRadians(0),1.0,true,0));
            path.add(new StopWaypoint(60, -100, Math.toRadians(-90),1.0));
        }

        c.followPath(path,40,this);
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
