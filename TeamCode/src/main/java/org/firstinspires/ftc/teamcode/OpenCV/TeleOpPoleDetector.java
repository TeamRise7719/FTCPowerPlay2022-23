package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Created by Sean Cardosi on 2/9/23.
 */
public class TeleOpPoleDetector {
    private OpenCvWebcam frontCam;
    private OpenCvWebcam backCam;
    private PoleDetectionPipeline opencvfront = null;
    private PoleDetectionPipeline opencvback = null;
    Telemetry telemetry;

    public TeleOpPoleDetector(OpMode opMode, Telemetry telemetry) {
        //initialize webcam
        this.telemetry = telemetry;
        frontCam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Front Camera"));
        backCam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Back Camera"));
    }

    public void findPole() {

        opencvfront = new PoleDetectionPipeline(telemetry);
        opencvback = new PoleDetectionPipeline(telemetry);
        frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontCam.setPipeline(opencvfront);
                frontCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
                frontCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        backCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                backCam.setPipeline(opencvback);
                backCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
                backCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }
    public double frontRectWidth(){
        return Math.min(opencvfront.rotatedRect.size.width, opencvfront.rotatedRect.size.height);
    }
    public double backRectWidth(){
        return Math.min(opencvback.rotatedRect.size.width, opencvback.rotatedRect.size.height);
    }

    public void stopCameras() {
        frontCam.stopStreaming();
        backCam.stopStreaming();
    }

    public double frontForwardDistance() {
        return opencvfront.DF;
    }
    public double backForwardDistance() {
        return opencvback.DF;
    }


    public double frontSideDistance() {
        return opencvfront.DS;
    }
    public double backSideDistance() {
        return opencvback.DS;
    }


//    public void showAverageFrameTime() {
//        telemetry.addData("Average Frame Process Time (ms)", frontCam.getTotalFrameTimeMs());
//    }
//    public void showFPS() {
//        telemetry.addData("FPS", frontCam.getFps());
//    }
}
