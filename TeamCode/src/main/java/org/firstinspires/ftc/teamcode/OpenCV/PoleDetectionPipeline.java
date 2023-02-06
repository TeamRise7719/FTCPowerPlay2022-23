package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

/**
 * Created by Sean Cardosi on 1/16/23.
 */
public class PoleDetectionPipeline extends OpenCvPipeline {

    Mat yCrCb = new Mat();
    Mat binaryMat = new Mat();
    Rect poleRect = new Rect();
    Scalar TEXT_COLOR = new Scalar(0,0,0);
    Scalar HORIZON_COLOR = new Scalar(0,255,0);
    Scalar CONTOUR_COLOR = new Scalar(210,220,50);
    ArrayList<MatOfPoint> contours = new ArrayList<>();
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
    Scalar poleLower = new Scalar(60,135,10);
    Scalar poleHigher = new Scalar(190,180,105);
    Telemetry telemetry;
    RotatedRect rotatedRect;

    double PHYSICAL_DISTANCE = 15;//cm
    double PHYSICAL_WIDTH = 2.5;//cm

    double FOCAL_LENGTH = 540;

    public PoleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(yCrCb, yCrCb, kernel);

        Core.inRange(yCrCb,poleLower,poleHigher,binaryMat);
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < 5);
        Imgproc.drawContours(input, contours, -1, CONTOUR_COLOR);
        rotatedRect = new RotatedRect();

        if (!contours.isEmpty()) {
            MatOfPoint biggestPole = Collections.max(contours,Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).height));
            if (Imgproc.contourArea(biggestPole) > 250) {//min area to be a pole prevents reading wrong objects as poles and makes the box consistent
                poleRect = Imgproc.boundingRect(biggestPole);
                MatOfPoint2f  h = new MatOfPoint2f();
                biggestPole.convertTo(h, CvType.CV_32F);
                rotatedRect = Imgproc.minAreaRect(h);
                Imgproc.rectangle(input, poleRect, CONTOUR_COLOR, 2);
                Imgproc.putText(input, "Pole " + (poleRect.x + (poleRect.width / 2.0)) + "," + (poleRect.y + (poleRect.height / 2.0)),new Point(poleRect.x,poleRect.y < 10 ? (poleRect.y + poleRect.height+20) : (poleRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR,2);
                Imgproc.circle(input, new Point(poleRect.x + (poleRect.width / 2.0), poleRect.y + (poleRect.height / 2.0)), 3, HORIZON_COLOR,3);
            }
        }

//        telemetry.addData("Distance: ", 1.0 / ((poleRect.width * 0.0095 * 0.367) / (2.5)));
//        telemetry.addData("Better Distance: ", 1.0 / (((Math.min(rotatedRect.size.width, rotatedRect.size.height)) * 0.0095 * 0.367) / (2.5)));
//
//        telemetry.addData("H-value at 10cm ", 10 * 2.5 / ((poleRect.width * 0.367)));
//        telemetry.update();

        double F = (rotatedRect.size.width * PHYSICAL_DISTANCE) / PHYSICAL_WIDTH;
        telemetry.addData("Calculated Camera Focal Length",F);

        double D = (PHYSICAL_WIDTH * FOCAL_LENGTH) / rotatedRect.size.width;
        telemetry.addData("Distance to Pole", D);

        telemetry.update();

        contours.clear();
        yCrCb.release();
        binaryMat.release();
        return input;
    }

    public double rectWidth() {
        return rotatedRect.size.width;
    }
}