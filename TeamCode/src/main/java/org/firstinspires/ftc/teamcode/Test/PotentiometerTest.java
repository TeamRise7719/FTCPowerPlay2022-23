package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Sean Cardosi on 1/27/23.
 */
@TeleOp
public class PotentiometerTest extends OpMode {

    AnalogInput pot;

    @Override
    public void init() {
        pot = hardwareMap.analogInput.get("pot");
    }

    @Override
    public void loop() {
        telemetry.addData("Pot value",pot.getVoltage());
        telemetry.addData("Pot Max Value",pot.getMaxVoltage());
    }
}
