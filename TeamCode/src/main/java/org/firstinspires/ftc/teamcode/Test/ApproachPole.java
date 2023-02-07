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
    //    MotionController c;
    double sideTarget = 0;//pixels
    double forwardTarget = 10;//cm
    SeansSynchronousPID forwardPID;
    SeansSynchronousPID sidePID;
    double forwardP = 0.01;
//    double sideP = 0.0001;
    List<LynxModule> allHubs;
    Localizer l;
    SeanDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {

        d = new PoleDetector(this,telemetry);
//        c = new MotionController(hardwareMap);
        l = new Localizer(hardwareMap,new Pose(0,0,0));
        drive = new SeanDrivetrain(hardwareMap);
        forwardPID = new SeansSynchronousPID(forwardP,0,0);
//        sidePID = new SeansSynchronousPID(sideP,0,0);
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
//                double sideError = sidePID.calculateUseError(sideTarget - d.centerX());
                double sideError = forwardPID.calculateUseError(sideTarget - d.centerX());
                double forwardError = forwardPID.calculateUseError(forwardTarget - d.distance());
                double rError = forwardPID.calculateUseError(-pose.getHeading() * 10);
                drive.setMotorPowers(-forwardError,-sideError,rError,1.0);
            } else {
                drive.stopMotors();
            }
        }
    }
}
