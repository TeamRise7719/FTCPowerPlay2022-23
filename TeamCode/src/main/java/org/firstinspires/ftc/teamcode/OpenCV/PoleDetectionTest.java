package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Sean Cardosi on 1/16/23.
 */
@TeleOp
public class PoleDetectionTest extends LinearOpMode {

    PoleDetector d;

    double PHYSICAL_DISTANCE = 15;//cm
    double PHYSICAL_WIDTH = 2.5;//cm

    double FOCAL_LENGTH = 540;
    @Override
    public void runOpMode() throws InterruptedException {

        d = new PoleDetector(this,telemetry);

        d.findPole();

        waitForStart();

        while (!isStopRequested()) {
            double F = (d.rectWidth() * PHYSICAL_DISTANCE) / PHYSICAL_WIDTH;
            telemetry.addData("Calculated Camera Focal Length",F);

            double D = (PHYSICAL_WIDTH * FOCAL_LENGTH) / d.rectWidth();
            telemetry.addData("Distance to Pole", D);

            telemetry.update();
        }

    }
}
