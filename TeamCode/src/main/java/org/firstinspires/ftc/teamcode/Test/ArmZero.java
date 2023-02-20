package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

/**
 * Created by Sean Cardosi on 1/13/23.
 */
@TeleOp
public class ArmZero extends OpMode {

    SeansComponent c;

    @Override
    public void init() {
        c = new SeansComponent(hardwareMap);
        c.init();
    }

    @Override
    public void loop() {
//        c.setArm(GlobalVariables.back45);
    }
}
