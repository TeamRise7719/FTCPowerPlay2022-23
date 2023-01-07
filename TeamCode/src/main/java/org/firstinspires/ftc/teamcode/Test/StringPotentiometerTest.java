package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Sean Cardosi on 12/29/22.
 */
@Disabled
@TeleOp(name = "String Potentiometer", group = "Test")
public class StringPotentiometerTest extends OpMode {

    AnalogInput stringy;

    @Override
    public void init() {
        stringy = hardwareMap.analogInput.get("stringy");
    }

    @Override
    public void loop() {
        telemetry.addData("Stringy",stringy.getVoltage());
    }
}
