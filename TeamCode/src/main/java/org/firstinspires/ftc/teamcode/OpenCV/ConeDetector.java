package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Created by Sean Cardosi on 1/23/23.
 */
public class ConeDetector {
    private OpenCvWebcam webcam;
    private ConeDetectionPipeline opencv = null;
    Telemetry telemetry;

    public ConeDetector(LinearOpMode opMode, Telemetry telemetry) {
        //initialize webcam
        this.telemetry = telemetry;
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Back Camera"));
    }

    public void findPole() {

        opencv = new ConeDetectionPipeline(telemetry);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(opencv);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public double redConeCenterX() {
        return opencv.redRect.x + opencv.redRect.width / 2.0;
    }

    public double redConeCenterY() {
        return opencv.redRect.y + opencv.redRect.height / 2.0;
    }

    public double blueConeCenterX() {
        return opencv.blueRect.x + opencv.blueRect.width / 2.0;
    }

    public double blueConeCenterY() {
        return opencv.blueRect.y + opencv.blueRect.height / 2.0;
    }

    public void stopCamera() {
        webcam.stopStreaming();
    }
}
