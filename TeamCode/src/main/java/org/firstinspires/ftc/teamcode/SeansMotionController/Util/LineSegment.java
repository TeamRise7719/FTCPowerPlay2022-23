package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

import java.util.ArrayList;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * Created by Sean Cardosi on 9/5/22.
 */
public class LineSegment {

    public Point p0;
    public Point p1;
    double x0;
    double y0;
    double x1;
    double y1;


    public LineSegment(Point p0, Point p1) {
        this.p0 = p0;
        this.p1 = p1;
        x0 = this.p0.x;
        y0 = this.p0.y;
        x1 = this.p1.x;
        y1 = this.p1.y;
    }

    /**
     * Finds the points of intersection
     * @return
     */
    public ArrayList<Point> intersect(Point robot, double lookAheadDistance) {
        double Rx = robot.x;
        double Ry = robot.y;
        double m = slope();

        double a = 1 + pow(m,2);
        double b = (-2 * Rx) + (-2 * pow(m,2) * x0) + (2 * m * y0) + (-2 * Ry * m);
        double c = (pow(Rx,2)) + (pow(m,2) * pow(x0,2)) + (-2 * m * x0 * y0) + (2 * Ry * m * x0) + (pow(y0,2)) + (-2 * Ry * y0) + (pow(Ry,2)) - (pow(lookAheadDistance,2));
        double discriminant = pow(b,2) - 4 * a * c;

        double X0;
        double Y0;
        double X1;
        double Y1;
        if (!Double.isInfinite(m)) {//Non-vertical line
            if (discriminant == 0) {//One Solution
                X0 = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
                Y0 = m * (X0 - x0) + y0;
                X1 = Double.NaN;
                Y1 = Double.NaN;
            } else if (discriminant > 0) {//Two Solution
                X0 = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
                Y0 = m * (X0 - x0) + y0;
                X1 = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
                Y1 = m * (X1 - x0) + y0;
            } else {//No solution
                X0 = Double.NaN;
                Y0 = Double.NaN;
                X1 = Double.NaN;
                Y1 = Double.NaN;
            }
        } else {//Vertical Line
            Y0 = Ry + lookAheadDistance;
            Y1 = Ry - lookAheadDistance;
            X0 = x0;
            X1 = x1;
        }
        double segmentLength = distance(p0,p1);

        if (!Double.isNaN(X0) && (distance(new Point(X0,Y0), p0) > segmentLength || distance(new Point(X0,Y0), p1) > segmentLength)) {//(Check for NaN) Are the provided points on the segment? See by checking distance from endpoints and compare to segment length
            X0 = Double.NaN;
            Y0 = Double.NaN;
        }
        if (!Double.isNaN(X1) && (distance(new Point(X1,Y1), p0) > segmentLength || distance(new Point(X1,Y1), p1) > segmentLength)) {//(Check for NaN) Are the provided points on the segment? See by checking distance from endpoints and compare to segment length
            X1 = Double.NaN;
            Y1 = Double.NaN;
        }
        ArrayList<Point> intersections = new ArrayList<>();
        if (!Double.isNaN(X0)) {
            intersections.add(new Point(X0,Y0));
        }
        if (!Double.isNaN(X1)) {
            intersections.add(new Point(X1,Y1));
        }
        return intersections;
    }

    public double slope() { return ((y1 - y0) / (x1 - x0)); }

    public double distance(Point a, Point b) { return sqrt(pow((b.x - a.x),2) + pow((b.y-a.y),2)); }
}

