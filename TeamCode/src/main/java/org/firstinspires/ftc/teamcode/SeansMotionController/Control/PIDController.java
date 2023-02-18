package org.firstinspires.ftc.teamcode.SeansMotionController.Control;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sean Cardosi on 2/9/23.
 */
public class PIDController implements FeedbackController {

    PIDCoefficients coefficients;

    protected boolean hasRun = false;

    protected ElapsedTime timer = new ElapsedTime();

    protected double previousError = 0;

    protected double integralSum = 0;

    protected double derivative = 0;

    public PIDController(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    @Override
    public double calculate(double reference, double state) {
        double dt = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error,dt);
        integrate(error,dt);
        previousError = error;
        return error * coefficients.Kp + integralSum * coefficients.Ki + derivative * coefficients.Kd;
    }

    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.seconds();
        timer.reset();
        return dt;
    }

    protected double calculateError(double reference, double state) {
        return reference - state;
    }

    protected void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
    }

    protected double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
        return derivative;
    }
}
