package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCV.PoleDetector;
import org.firstinspires.ftc.teamcode.Qaqortoq.Subsystems.Sensing.SeansSynchronousPID;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.Localizer;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeanDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Pose;

import java.util.List;

/**
 * Created by Sean Cardosi on 2/6/23.
 */
@TeleOp
public class ApproachPole extends LinearOpMode {

    PoleDetector d;
    double FOCAL_LENGTH = 540;
    double PHYSICAL_WIDTH = 2.5;//cm
    //    MotionController c;
    double sideTarget = 180;//pixels
    double forwardTarget = 10;//cm
    SeansSynchronousPID pid;
    double P = 0.01;
    double I = 0.0;
    double D = 0.0;
    List<LynxModule> allHubs;
    Localizer l;
    SeanDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {

        d = new PoleDetector(this,telemetry);
//        c = new MotionController(hardwareMap);
        l = new Localizer(hardwareMap,new Pose(0,0,0));
        drive = new SeanDrivetrain(hardwareMap);
        pid = new SeansSynchronousPID(P,I,D);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        d.findPole();

        sleep(2000);

        telemetry.addData("Ready","");

        waitForStart();

        while (!this.isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            l.updatePose();
            Pose pose = l.getPose();
            if (!Double.isNaN(d.rectWidth())) {
//                double D = (PHYSICAL_WIDTH * FOCAL_LENGTH) / d.rectWidth();
                double sideError = sideTarget - d.centerX();
                double forwardError = forwardTarget - d.distance();
                double rError = 0 - pose.getHeading();
                drive.setMotorPowers(-sideError,-forwardError,rError,1.0);
            } else {
                drive.stopMotors();
            }
        }
    }
}
