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
import java.util.List;

/**
 * Created by Sean Cardosi on 1/23/23.
 */
public class ConeDetectionPipeline extends OpenCvPipeline {


    Telemetry telemetry;
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
    Mat YCrCb = new Mat();
    Mat maskRed = new Mat();
    Mat maskBlue = new Mat();


    // Red masking thresholding values
    public Scalar lowRed = new Scalar(0, 161, 60);
    public Scalar highRed = new Scalar(200, 255, 255);

    // Blue masking thresholding values
    public Scalar lowBlue = new Scalar(0, 80, 138);
    public Scalar highBlue = new Scalar(100, 255, 255);

    Rect redRect;
    Rect blueRect;

    double horizon = 130;

    List<MatOfPoint> redContours;
    List<MatOfPoint> blueContours;

    Scalar BLUE_CONTOUR_COLOR = new Scalar(100,0,255);
    Scalar RED_CONTOUR_COLOR = new Scalar(145,34,54);
    Scalar HORIZON_COLOR = new Scalar(0,255,0);
    Scalar TEXT_COLOR = new Scalar(0,0,0);

    int minArea = 250;


    public enum Colors {
        BLUE,
        RED,
        BOTH
    }

    Colors color = Colors.BOTH;

    public ConeDetectionPipeline(Telemetry telemetry) {

        redContours = new ArrayList<>();
        redRect = new Rect();

        blueContours = new ArrayList<>();
        blueRect = new Rect();

        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(YCrCb, YCrCb, kernel);

        RotatedRect redRotatedRect = new RotatedRect();

        if (color.equals(Colors.RED) || color.equals(Colors.BOTH)) {
            Core.inRange(YCrCb, lowRed, highRed, maskRed);

            redContours.clear();

            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            redContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);//Height at which we no longer consider a color a cone.
            Imgproc.drawContours(input, redContours, -1, RED_CONTOUR_COLOR);

            if(!redContours.isEmpty()) {
                MatOfPoint biggestRedContour = Collections.max(redContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                if(Imgproc.contourArea(biggestRedContour) > minArea) {
                    redRect = Imgproc.boundingRect(biggestRedContour);
                    MatOfPoint2f  red = new MatOfPoint2f();
                    biggestRedContour.convertTo(red, CvType.CV_32F);
                    redRotatedRect = Imgproc.minAreaRect(red);

                    Imgproc.rectangle(input, redRect, RED_CONTOUR_COLOR, 2);
                    // Imgproc.putText(input, "Red Cone", new Point(redRect.x, redRect.y < 10 ? (redRect.y+redRect.height + 20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
                    Imgproc.putText(input, "Red Cone: " + (redRect.x + (redRect.width / 2.0)) + "," + (redRect.y + (redRect.height / 2.0)),new Point(redRect.x,redRect.y < 10 ? (redRect.y + redRect.height+20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR,2);
                    Imgproc.circle(input, new Point(redRect.x + (redRect.width / 2.0), redRect.y + (redRect.height / 2.0)), 3, HORIZON_COLOR,3);
                }
            }

            maskRed.release();
        }
        RotatedRect blueRotatedRect = new RotatedRect();

        if (color.equals(Colors.BLUE) || color.equals(Colors.BOTH)) {
            Core.inRange(YCrCb, lowBlue, highBlue, maskBlue);

            blueContours.clear();

            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            blueContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);//Height at which we no longer consider a color a cone.
            Imgproc.drawContours(input, blueContours, -1, BLUE_CONTOUR_COLOR);

            if(!blueContours.isEmpty()) {
                MatOfPoint biggestBlueContour = Collections.max(blueContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                if(Imgproc.contourArea(biggestBlueContour) > minArea) {
                    blueRect = Imgproc.boundingRect(biggestBlueContour);
                    MatOfPoint2f  blue = new MatOfPoint2f();
                    biggestBlueContour.convertTo(blue, CvType.CV_32F);
                    blueRotatedRect = Imgproc.minAreaRect(blue);

                    Imgproc.rectangle(input, blueRect, BLUE_CONTOUR_COLOR, 2);
                    // Imgproc.putText(input, "Blue Cone", new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
                    Imgproc.putText(input, "Blue Cone: " + (blueRect.x + (blueRect.width / 2.0)) + "," + (blueRect.y + (blueRect.height / 2.0)),new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y + blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR,2);
                    Imgproc.circle(input, new Point(blueRect.x + (blueRect.width / 2.0), blueRect.y + (blueRect.height / 2.0)), 3, HORIZON_COLOR,3);
                }
            }
            maskBlue.release();
        }
//        telemetry.addData("Blue Distance: ", 1.0 / (((Math.min(blueRotatedRect.size.width, blueRotatedRect.size.height)) * 0.0267 * 0.367) / (2.5)));
//
//        telemetry.addData("Red Distance: ", 1.0 / (((Math.min(redRotatedRect.size.width, redRotatedRect.size.height)) * 0.0267 * 0.367) / (2.5)));
//
//        telemetry.addData("H-value at 10cm ", 40 * 2.5 / ((blueRect.width * 0.367)));
//        telemetry.update();

        // Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);
        YCrCb.release();

        return input;
    }
}