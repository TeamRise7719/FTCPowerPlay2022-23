package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

/**
 * Created by Sean Cardosi on 1/14/23.
 */
public class ArmState extends StateMachineThread {

    SeansComponent c;

    public enum States {
        FRONT45,
        FRONT90,
        FRONT135,
        UP,
        BACK135,
        BACK90,
        BACK45,
        OPEN,
        CLOSE,
        NOTHING,
        OPENM
    }

    LinearOpMode opMode;

    public ArmState(String startingState, LinearOpMode opMode, HardwareMap hardwareMap) {
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
                case FRONT45:
                    c.setArm(GlobalVariables.front45);
                    break;
                case FRONT90:
                    break;
                case FRONT135:
                    c.setArm(GlobalVariables.front135);
                    break;
                case UP:
                    c.setArm(GlobalVariables.up);
                    break;
                case BACK135:
                    break;
                case BACK90:
                    c.setArm(GlobalVariables.back90);
                    break;
                case BACK45:
                    c.setArm(GlobalVariables.back45);
                    break;
                case OPEN:
                    c.open();
                    break;
                case CLOSE:
                    c.close();
                    break;
                case OPENM:
                    c.openM();
                    break;
            }
        }
    }

    public void setState(String state) {
        super.setState(state);
    }
}
