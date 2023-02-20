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
    double lastPosition;
    double armTarget;
    double armFFAngle;
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
        switch (States.valueOf(super.state)) {
            case NOTHING:
                break;
            case FRONT45:
                armTarget = GlobalVariables.front45;
                lastPosition = GlobalVariables.front45;
                armFFAngle = 135;
                break;
            case FRONT90:
                armTarget = GlobalVariables.front90;
                lastPosition = GlobalVariables.front90;
                armFFAngle = 180;
                break;
            case FRONT135:
                armTarget = GlobalVariables.front135;
                lastPosition = GlobalVariables.front135;
                armFFAngle = 225;
                break;
            case UP:
                armTarget = GlobalVariables.up;
                lastPosition = GlobalVariables.up;
                armFFAngle = 270;
                break;
            case BACK135:
                armTarget = GlobalVariables.back135;
                lastPosition = GlobalVariables.back135;
                armFFAngle = 315;
                break;
            case BACK90:
                armTarget = GlobalVariables.back90;
                lastPosition = GlobalVariables.back90;
                armFFAngle = 0;
                break;
            case BACK45:
                armTarget = GlobalVariables.back45;
                lastPosition = GlobalVariables.back45;
                armFFAngle = 405;
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

    @Override
    public void run() {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean onWayUp = false;
        double kcos = -0.1;

        while (!opMode.isStopRequested()) {
            switch (States.valueOf(super.state)) {
                case NOTHING:
                    break;
                case FRONT45:
                    lastPosition = armTarget;
                    armTarget = GlobalVariables.front45;
                    armFFAngle = 135;
                    onWayUp = false;
                    //Can never raise to the lowest
                    break;
                case FRONT90:
                    lastPosition = armTarget;
                    armTarget = GlobalVariables.front90;
                    armFFAngle = 180;
                    onWayUp = lastPosition == GlobalVariables.front45;
                    break;
                case FRONT135:
                    lastPosition = armTarget;
                    armTarget = GlobalVariables.front135;
                    armFFAngle = 225;
                    onWayUp = lastPosition == GlobalVariables.front45 || lastPosition == GlobalVariables.front90;
                    break;
                case UP:
                    lastPosition = armTarget;
                    armTarget = GlobalVariables.up;
                    armFFAngle = 270;
                    onWayUp = true;//TODO: May need to be false if unstable
                    break;
                case BACK135:
                    lastPosition = armTarget;
                    armTarget = GlobalVariables.back135;
                    armFFAngle = 315;
                    onWayUp = lastPosition == GlobalVariables.back45 || lastPosition == GlobalVariables.back90;
                    break;
                case BACK90:
                    lastPosition = armTarget;
                    armTarget = GlobalVariables.back90;
                    armFFAngle = 0;
                    onWayUp = lastPosition == GlobalVariables.back45;
                    break;
                case BACK45:
                    lastPosition = armTarget;
                    armTarget = GlobalVariables.back45;
                    armFFAngle = 405;
                    onWayUp = false;
                    //Can never raise to the lowest
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
            double ffOutput;
            if (armTarget != GlobalVariables.up) {
                ffOutput = kcos * Math.cos((Math.toRadians(armFFAngle)));
            } else {
                ffOutput = 0.0;
            }

            double pidOutput;
            if (onWayUp) {
                pidOutput = goingUp.calculate(armTarget, pot.getVoltage());
            } else {
                pidOutput = armPID.calculate(armTarget, pot.getVoltage());
            }
            armMotor.setPower(pidOutput + ffOutput);
        }
    }

    public void setState(String state) {
        super.setState(state);
    }
}
