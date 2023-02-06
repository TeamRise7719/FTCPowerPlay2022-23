package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Created by Sean Cardosi on 1/16/23.
 */
public class PoleDetector {
    private OpenCvWebcam webcam;
    private PoleDetectionPipeline opencv = null;
    Telemetry telemetry;

    public PoleDetector(LinearOpMode opMode, Telemetry telemetry) {
        //initialize webcam
        this.telemetry = telemetry;
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Front Camera"));
    }

    public void findPole() {

        opencv = new PoleDetectionPipeline(telemetry);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(opencv);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }
    public double rectWidth(){
        return Math.min(opencv.rotatedRect.size.width,opencv.rotatedRect.size.height);
    }

    public double centerX() {
        return opencv.poleRect.x;
    }

    public double centerY() {
        return opencv.poleRect.y;
    }

    public void stopCamera() {
        webcam.stopStreaming();
    }
}
