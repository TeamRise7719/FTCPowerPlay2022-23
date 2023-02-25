package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.MotionController;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.Odometry;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeanDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.StopWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Waypoint;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Drive PID")
public class TestingPIDAuto  extends LinearOpMode {

    MotionController c;
    SeanDrivetrain d;
    SeansComponent component;
    Odometry o;


    @Override
    public void runOpMode() throws InterruptedException {
        c = new MotionController(hardwareMap);
        d = new SeanDrivetrain(hardwareMap);
        d.resetHeading();

        component = new SeansComponent(hardwareMap);
        component.init();
        component.odoServo.setPosition(GlobalVariables.odoDown);

        waitForStart();

        ArrayList<Waypoint> path = new ArrayList<>();
         path.add(new StopWaypoint(10,0, Math.toRadians(0),1));
        c.followPath(path,40,this);
    }
}
