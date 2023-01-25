package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.ConeDetector;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeanDrivetrain;

/**
 * Created by Sean Cardosi on 1/23/23.
 */
@TeleOp
public class TurnToCone extends LinearOpMode {

    private ConeDetector coneDetector;
    SeanDrivetrain drive;
    SeansSynchronousPID pid;
    double P = 0.003;
    double I = 0.0;
    double D = 0.003;

    @Override
    public void runOpMode() throws InterruptedException {

        coneDetector = new ConeDetector(this,telemetry);
        drive = new SeanDrivetrain(hardwareMap);
        pid = new SeansSynchronousPID();
        pid.setPID(P,I,D);
        pid.setOutputRange(-100,100);
        coneDetector.findPole();
        double error = 320 - coneDetector.redConeCenterX();

        waitForStart();

        sleep(500);

        while (!isStopRequested()) {

            error = 320 - coneDetector.redConeCenterX();
            double r = -pid.calculateUseError(error);

            if (!Double.isNaN(coneDetector.redConeCenterX())) {
                if (Math.abs(error) > 15) {
                    drive.setMotorPowers(0, 0, r, 1, telemetry);
                    telemetry.addData("Error",error);
                } else {
                    drive.stopMotors();
                    telemetry.addData("Stopping","");
                }
            } else {
                drive.stopMotors();
                telemetry.addData("Cant find","");
            }
            telemetry.update();
        }
    }
}
