package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Vision.Subsystems.Components.LiftDistance;
import org.firstinspires.ftc.teamcode.Vision.Subsystems.Driving.Drivetrain;

/**
 * Created by Sean Cardosi on 8/28/22.
 */
@Disabled
@TeleOp(name = "Qaqortoq Drive", group = "Qaqortoq")
public class V2DriveTest extends OpMode{

    private Drivetrain drivetrain;
    boolean isReady = false;

    DcMotor liftL, liftR;

    Servo right, left, clawR, clawL;

    double positArm = 0.625;
    double positClaw = 0.45;
    double middle = 0.44;
    double backDown = 0.625;
    double backUp = 0.55;
    double frontDown = 0.27;
    double frontUp = 0.34;
    float power = 0;

    boolean bumperChangedL = false, bumperChangedR = false;
    boolean armIsFront = false, armIsUp = false;
    boolean buttonChanged = false, clawOpen = false;

    LiftDistance liftD;

    @Override
    public void init() {

        //Initialize robot
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.runUsingEncoders();

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");
        clawR = hardwareMap.get(Servo.class, "clawR");
        clawL = hardwareMap.get(Servo.class, "clawL");

        liftL = hardwareMap.dcMotor.get("leftEncoder");
        liftL.setDirection(DcMotor.Direction.REVERSE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(Servo.Direction.FORWARD);
        left.setDirection(Servo.Direction.REVERSE);
        clawR.setDirection(Servo.Direction.FORWARD);
        clawL.setDirection(Servo.Direction.REVERSE);

        clawR.setPosition(positClaw);
        clawL.setPosition(positClaw);

        liftD = new LiftDistance(hardwareMap);

//        right.setPosition(posit);
//        left.setPosition(posit);
        isReady = true;

    }

    @Override
    public void init_loop() {
        if (isReady) {
            telemetry.addData(">", "Robot Ready!");
            telemetry.update();
        }

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

        drivetrain.drive(gamepad1, telemetry);

        //Puts robot into starting position
        if (gamepad1.x) {
            drivetrain.resetHeading();
        }

        //Raises and lowers arm between 90 degree and down positions
        if (gamepad2.right_bumper && !bumperChangedR) {
            if (!armIsUp) {
                armIsUp = true;
            } else {
                armIsUp = false;
            }
            bumperChangedR = true;
        } else if (!gamepad2.right_bumper) {
            bumperChangedR = false;
        }
        // Flips arm
        if (gamepad2.left_bumper && !bumperChangedL && !clawOpen) {
            if (!armIsFront) {
                armIsFront = true;
            } else {
                armIsFront = false;
            }
            bumperChangedL = true;
        } else if (!gamepad2.left_bumper) {
            bumperChangedL = false;
        }
        //Opens and closes claw
        if (gamepad2.a && !buttonChanged) {
            if (!clawOpen) {
                clawOpen = true;
            } else {
                clawOpen = false;
            }
            buttonChanged = true;
        } else if (!gamepad2.a) {
            buttonChanged = false;
        }
        //Puts arm upright
        if (!clawOpen && gamepad2.dpad_up) {
             positArm = middle;
        } else {
            if (armIsUp && armIsFront) {
                positArm = frontUp;
            }
            if (armIsUp && !armIsFront) {
                positArm = backUp;
            }
            if (!armIsUp && armIsFront) {
                positArm = frontDown;
            }
            if (!armIsUp && !armIsFront) {
                positArm = backDown;
            }
        }

        //Claw's open and close positions
        if (clawOpen) {
            positClaw = 0.5; // + ( (gamepad2.right_trigger - gamepad2.left_trigger) / 8);
        }
        if (!clawOpen ) {
            positClaw = 0.26; // + ( (gamepad2.right_trigger - gamepad2.left_trigger) / 8);
        }

        //Moves the lift
        power = gamepad2.right_stick_y;

        liftL.setPower(power);
        liftR.setPower(power); 

        //Makes both sides of the claws and arms move together
        if (armIsFront) {
            right.setPosition(positArm);
            left.setPosition(positArm);
        } else {
            right.setPosition(positArm);
            left.setPosition(positArm );
        }

        //Lift positions
//        if (gamepad2.b){
//           low pole
//        }
//
//        if (gamepad2.y){
//            middle pole
//        }
//
//        if (gamepad2.x){
//            high pole
//        }

        clawR.setPosition(positClaw);
        clawL.setPosition(positClaw);

        telemetry.addData(">", "Claw Position: %f", positClaw );



    }
}
