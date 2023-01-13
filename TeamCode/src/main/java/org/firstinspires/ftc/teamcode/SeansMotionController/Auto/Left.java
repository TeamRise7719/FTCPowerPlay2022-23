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
 * Created by Sean Cardosi on 1/13/23.
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
            while (e1.seconds() < 5 && !this.isStopRequested()) {
                component.liftTo(62);
            }
            e1.reset();
            while (e1.seconds() < 2.2 && !this.isStopRequested()) {
                component.liftTo(12);
            }
            e1.reset();
            while (e1.seconds() < 5 && !this.isStopRequested()) {
                component.liftTo(65);
            }
        };


        Runnable arm1Thread = () -> {
            ElapsedTime e1 = new ElapsedTime();
            e1.reset();
            component.setArm(0.53);
            while (e1.seconds() < 1.1) {}
            component.setClaw(0.525);
            e1.reset();

            while (e1.seconds() < 0.6) {}
            component.setClaw(0.56);
            component.setArm(0.28);
            e1.reset();

            while (e1.seconds() < 1.5) {}
            component.setClaw(0.525);

            e1.reset();
            while (e1.seconds() < 0.85) {}
            component.setClaw(0.56);

            e1.reset();
            while (e1.seconds() < 1) {}
            component.setArm(0.53);

            e1.reset();
            while (e1.seconds() < 2) {}
            component.setClaw(0.525);

            e1.reset();
            while (e1.seconds() < 0.25) {}
            component.setClaw(0.56);
            component.setArm(0.28);
        };

        waitForStart();

        component.setClaw(0.56);

        ArrayList<Waypoint> path = new ArrayList<>();
        ArrayList<ActionPoint> thingsToDo = new ArrayList<>();
        thingsToDo.add(new ActionPoint(new Point(-8,0),10,new Thread(lift1Thread)));
        thingsToDo.add(new ActionPoint(new Point(-8,-120),20,new Thread(arm1Thread)));


        path.add(new HeadingControlledWaypoint(-8, -20, Math.toRadians(0),true,1));
        path.add(new HeadingControlledWaypoint(-8, -140, Math.toRadians(180),false,3));
        path.add(new HeadingControlledWaypoint(-12, -147, Math.toRadians(138),true,1));
        path.add(new Wait(1100));
        path.add(new HeadingControlledWaypoint(-2, -93, Math.toRadians(180),false,1));
        path.add(new HeadingControlledWaypoint(-75, -128, Math.toRadians(174),true,1));
        path.add(new Wait(1100));
        path.add(new HeadingControlledWaypoint(-8, -130, Math.toRadians(180),false,3));
        path.add(new HeadingControlledWaypoint(-2, -141.5, Math.toRadians(140),false,3));
        path.add(new HeadingControlledWaypoint(-12, -147, Math.toRadians(140),true,1));
        path.add(new Wait(1000));
        path.add(new StopWaypoint(-10, -140, Math.toRadians(180)));



        c.followPath(path,thingsToDo,40,true,this, telemetry);
    }
}
