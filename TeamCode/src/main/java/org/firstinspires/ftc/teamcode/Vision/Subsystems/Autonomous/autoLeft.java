package org.firstinspires.ftc.teamcode.Vision.Subsystems.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Subsystems.Components.Component;
import org.firstinspires.ftc.teamcode.Vision.Subsystems.Components.LiftDistance;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansEncLibrary;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
@Autonomous(name = " Vision Auto Left", group = "Vision Autos")
public class autoLeft extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    SeansEncLibrary enc;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // We will need to do our own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Tag1 = 1; // Tag ID 1 from the 36h11 family
    int Tag2 = 2; // Tag ID 2 from the 36h11 family
    int Tag3 = 3; // Tag ID 3 from the 36h11 family
    int detectedTag = 0;


    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

                if (tagOfInterest == null) {
                    telemetry.addLine("(A tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //-------------------------------+=(Auto)=+-------------------------------\\


        Component program = new Component(hardwareMap);
        LiftDistance liftl = new LiftDistance(hardwareMap);

        Runnable liftAction = () -> {liftl.liftD(24);};
        Thread liftThread = new Thread(liftAction);

        program.init() ;
        liftl.init();
        // Actually do something useful *
        if (tagOfInterest.id == Tag1){//Position 1: The left-most parking zone
            /*
             * Insert auto code here for position 1
             */
            program.moveLift(.1);
            sleep(1000);
            program.stopLift();
            program.grab();
           liftl.liftD(4);
            liftThread.start();
            enc.steeringDrive(-4, false, true);
            enc.steeringDrive(3,false,false);
            enc.arcTurn(-90);
            enc.steeringDrive(35,false,true);
            while (liftThread.isAlive()){}
            enc.steeringDrive(-2.5,false,false);
            liftl.liftD(-20);
            program.release();
            enc.steeringDrive(2.5,false,false);
            enc.steeringDrive(-13, false,true);
            enc.steeringDrive(23,false,false);


            /*
             * Negative values will turn counterclockwise or strafe left or go backwards depending on
             * what is specified in the function parameters.
             */
        } else if (tagOfInterest.id == Tag2) {//Position 2: The middle parking zone
            /*
             * Insert auto code here for position 1
             */
            program.moveLift(.1);
            sleep(1000);
            program.stopLift();
            program.grab();
            liftl.liftD(4);
            liftThread.start();
            enc.steeringDrive(-4, false, true);
            enc.steeringDrive(3,false,false);
            enc.arcTurn(-90);
            enc.steeringDrive(35,false,true);
            while (liftThread.isAlive()){}
            enc.steeringDrive(-2.5,false,false);
            liftl.liftD(-20);
            program.release();
            enc.steeringDrive(2.5,false,false);



        } else if (tagOfInterest.id == Tag3) {//Position 3: The right-most parking zone
            /*
             * Insert auto code here for position 1
             */
            program.moveLift(.1);
            sleep(1000);
            program.stopLift();
            program.grab();
            liftl.liftD(4);
            liftThread.start();
            enc.steeringDrive(-4, false, true);
            enc.steeringDrive(3,false,false);
            enc.arcTurn(-90);
            enc.steeringDrive(35,false,true);
            while (liftThread.isAlive()){}
            enc.steeringDrive(-2.5,false,false);
            //liftDownThread.start();
            liftl.liftD(-20);
            program.release();
            enc.steeringDrive(2.5,false,false);
            enc.steeringDrive(-13, false,true);
            enc.steeringDrive(-23,false,false);
            enc.arcTurn(90);
        } else {
            /*
             * Insert default auto code here since we never found the tag.
             */
        }



        //-------------------------------+=(Auto)=+-------------------------------\\
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
