package org.firstinspires.ftc.teamcode.SeansMotionController.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SeansMotionController.Control.MotionController;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeanDrivetrain;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.ActionPoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Point;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.StopWaypoint;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Wait;
import org.firstinspires.ftc.teamcode.SeansMotionController.Util.Waypoint;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 1/11/23.
 */
@TeleOp(name = "Left", group = "Sean Auto")
public class Left extends LinearOpMode {

    MotionController c;
    SeanDrivetrain d;
    SeansComponent component;

    @Override
    public void runOpMode() throws InterruptedException {
        c = new MotionController(hardwareMap);
        d = new SeanDrivetrain(hardwareMap);
        component = new SeansComponent(hardwareMap);
        component.init();
        d.resetHeading();

        Runnable lift1Thread = () -> {
            ElapsedTime e1 = new ElapsedTime();
            e1.reset();
            while (e1.seconds() < 6) {
                component.liftTo(62);
            }
            e1.reset();
            while (e1.seconds() < 5) {
                component.liftTo(12.5);
            }
            e1.reset();
            while (e1.seconds() < 5) {
                component.liftTo(62);
            }
        };


        Runnable arm1Thread = () -> {
            ElapsedTime e1 = new ElapsedTime();
            e1.reset();
            component.setArm(0.53);
            while (e1.seconds() < 1.5) {}
            component.setClaw(0.48);
            e1.reset();

            while (e1.seconds() < 1) {}
            component.setClaw(0.525);
            component.setArm(0.28);
            e1.reset();

            while (e1.seconds() < 1) {}
            component.setClaw(0.48);
            e1.reset();
            while (e1.seconds() < 1) {}
            component.setClaw(0.525);

            e1.reset();
            while (e1.seconds() < 1) {}
            component.setArm(0.53);
        };

        waitForStart();

        component.setClaw(0.525);

        ArrayList<Waypoint> path = new ArrayList<>();
        ArrayList<ActionPoint> thingsToDo = new ArrayList<>();
        thingsToDo.add(new ActionPoint(new Point(-8,0),10,new Thread(lift1Thread)));
        thingsToDo.add(new ActionPoint(new Point(-8,-120),10,new Thread(arm1Thread)));


        path.add(new HeadingControlledWaypoint(-8, 0, Math.toRadians(0)));
        path.add(new HeadingControlledWaypoint(-8, -120, Math.toRadians(180)));
        path.add(new HeadingControlledWaypoint(-2, -143, Math.toRadians(135)));
        path.add(new Wait(-2, -143, Math.toRadians(135),1200));
        path.add(new HeadingControlledWaypoint(-8, -130, Math.toRadians(180)));
        path.add(new HeadingControlledWaypoint(-69.5, -130, Math.toRadians(180)));
        path.add(new Wait(-2, -143, Math.toRadians(135),1500));
        path.add(new HeadingControlledWaypoint(-8, -130, Math.toRadians(180)));
        path.add(new HeadingControlledWaypoint(-2, -143, Math.toRadians(135)));
        path.add(new Wait(-2, -143, Math.toRadians(135),1500));
        path.add(new StopWaypoint(-2, -143, Math.toRadians(135)));



        c.followPath(path,thingsToDo,40,true,this, telemetry);
    }
}
