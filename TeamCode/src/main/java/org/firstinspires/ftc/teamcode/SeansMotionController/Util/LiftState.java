package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

/**
 * Created by Sean Cardosi on 1/14/23.
 */
public class LiftState extends StateMachineThread {

    SeansComponent c;

    public enum States {
        NOTHING,
        RAISE40,
        RAISE42,
        RAISE11_5,
        RAISE8_5,
        RAISE6,
        RAISE3,
        RAISE0
    }

    LinearOpMode opMode;

    public LiftState(String startingState, LinearOpMode opMode, HardwareMap hardwareMap) {
        super(startingState);
        this.opMode = opMode;
        c = new SeansComponent(hardwareMap);
        c.init();
    }

    @Override
    public void run() {
        while (!opMode.isStopRequested()) {
            switch (States.valueOf(super.state)) {
                case NOTHING:
                    break;
                case RAISE40:
                    c.liftTo(40);
                    break;
                case RAISE11_5:
                    c.liftTo(11.5);
                    break;
                case RAISE8_5:
                    c.liftTo(8.5);
                    break;
                case RAISE42:
                    c.liftTo(42);
                    break;
                case RAISE6:
                    c.liftTo(6);
                    break;
                case RAISE0:
                    c.liftTo(0);
                    break;
                case RAISE3:
                    c.liftTo(3);
                    break;
            }
        }
    }

    public void setState(String state) {
        super.setState(state);
    }
}
