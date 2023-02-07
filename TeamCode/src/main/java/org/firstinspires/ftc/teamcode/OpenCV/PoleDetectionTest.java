package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Sean Cardosi on 1/16/23.
 */
@TeleOp
public class PoleDetectionTest extends LinearOpMode {

    PoleDetector d;

    double PHYSICAL_DISTANCE = 27.75;//cm
    double PHYSICAL_WIDTH = 2.5;//cm

    double FOCAL_LENGTH = 540;
    @Override
    public void runOpMode() throws InterruptedException {

        d = new PoleDetector(this,telemetry);

        d.findPole();

        waitForStart();

        while (!isStopRequested()) {}

    }
}
