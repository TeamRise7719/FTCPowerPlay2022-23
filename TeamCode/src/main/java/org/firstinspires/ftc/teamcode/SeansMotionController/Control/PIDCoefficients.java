package org.firstinspires.ftc.teamcode.SeansMotionController.Control;

/**
 * Created by Sean Cardosi on 2/9/23.
 */
public class PIDCoefficients {

    public double Kp;
    public double Ki;
    public double Kd;

    public PIDCoefficients(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
}
