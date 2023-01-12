package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 9/3/22.
 */
public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Subtract a point from this one
     */
    public Point add(Point p) {
        return new Point(this.x + p.x, this.y + p.y);
    }

    /**
     * Subtract a point from this one.
     */
    public Point minus(Point p) {
        return new Point(this.x - p.x, this.y - p.y);
    }

    /**
     * The r-value. The distance from the origin.
     */
    public double radius() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Simplified atan2 and less likely to mess up with (y,x) or (x,y)
     */
    public double atan() {
        return Math.atan2(y,x);
    }

    /**
     * Distance between a point and this one. Similar to vector calculation from to points.
     */
    public double distance(Point p) {
        return minus(p).radius();
    }

    /**
     * Rotates the point by the angle
     */
    public Point rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) - y * Math.cos(angle);
        return new Point(newX,newY);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    /**
     * Creates a new instance of a point with the same values.
     */
    public Point copy() {
        return new Point(x,y);
    }

    @Override
    public String toString() {
        return "(" + x + "," + y + ")";
    }
}