package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SeansMotionController.Control.Odometry;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class OdometryTelemetry extends OpMode {

    Odometry o;
    @Override
    public void init() {
        o = new Odometry(hardwareMap);
    }

    @Override
    public void loop() {
        ArrayList<List<Double>> odometryPos = new ArrayList<>();
        odometryPos.add(o.getWheelPositions());
        telemetry.addData("left", odometryPos.get(0));
    }
}
