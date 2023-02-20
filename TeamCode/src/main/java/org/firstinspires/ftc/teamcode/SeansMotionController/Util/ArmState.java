package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.SeansMotionController.Control.PIDController;
import org.firstinspires.ftc.teamcode.SeansMotionController.Drive.SeansComponent;

/**
 * Created by Sean Cardosi on 1/14/23.
 */
public class ArmState extends StateMachineThread {

    SeansComponent c;
    PIDController goingUp;
    PIDController armPID;
    AnalogInput pot;
    DcMotorEx armMotor;

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
        pot = hardwareMap.analogInput.get("pot");
        armMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goingUp = new PIDController(new PIDCoefficients(1.7,0,0));
        armPID = new PIDController(new PIDCoefficients(0.9,0,0.02));
    }

    @Override
    public void run() {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean onWayDown = false;
        boolean onWayUp = false;
        double lastPosition;
        double armTarget;
        double kcos = -0.1;
        double armFFAngle;

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
//            double ffOutput;
//            if (armTarget != GlobalVariables.up) {
//                ffOutput = kcos * Math.cos((Math.toRadians(armFFAngle)));
//            } else {
//                ffOutput = 0.0;
//            }
//            double pidOutput;
//            if (onWayUp) {
//                pidOutput = goingUp.calculate(armTarget, pot.getVoltage());
//            } else {
//                pidOutput = armPID.calculate(armTarget, pot.getVoltage());
//            }
//            armMotor.setPower(pidOutput + ffOutput);
        }
    }

    public void setState(String state) {
        super.setState(state);
    }
}
