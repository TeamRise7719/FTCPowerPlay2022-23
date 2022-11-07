package org.firstinspires.ftc.teamcode.Vision.Subsystems.Sensing;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

/**
 * Created by Sean Cardosi on 11/4/22.
 */
public class  SeansEncLibrary {

    private DcMotor left_back_drive;
    private DcMotor left_front_drive;
    private DcMotor right_back_drive;
    private DcMotor right_front_drive;

    private SynchronousPID turnPID;
    private SynchronousPID drivePID;



    public BNO055IMU gyro;
    private Orientation gyro_angle;

    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

    private double COUNTS_PER_MOTOR_REV = 537.6;
    private double EXTERNAL_GEAR_RATIO = 1.5;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    private double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415));

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public final double DRIVE_SPEED = 1;     // Nominal speed
    public final double DRIVE_SPEED_SLOW = 0.4;     // Slower speed for better accuracy.

    public final double TURN_SPEED = 1.0;     // Nominal half speed for better accuracy.

    private static final double HEADING_THRESHOLD = 0.05;      // As tight as we can make it with an integer gyro
    private static final int ENCODER_THRESHOLD = 3;      // As tight as we can make it with an integer gyro


    private static final double P_TURN_COEFF = 0.06;//0.008     // Larger is more responsive, but also less stable
    private static final double I_TURN_COEFF = 0.0;//0.0000000000015;//0.0000000000015  // Larger is more responsive, but also less stable
    private static final double D_TURN_COEFF = 0.0;//0.000001;//0.000001     // Larger is more responsive, but also less stable

    //11.11111111111% overshot
    private static final double P_DRIVE_COEFF = 1.0;    // Larger is more responsive, but also less stable
    private static final double I_DRIVE_COEFF = 0.7;/*0.0000000000059*/;     // Larger is more responsive, but also less stable
    private static final double D_DRIVE_COEFF = 0.01;/*0.00004*/;    // Larger is more responsive, but also less stable

    public SeansEncLibrary(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        left_back_drive = hardwareMap.dcMotor.get("backLeft");
        left_front_drive = hardwareMap.dcMotor.get("frontLeft");
        right_back_drive = hardwareMap.dcMotor.get("backRight");
        right_front_drive = hardwareMap.dcMotor.get("frontRight");

        telemetry = tel;
        linearOpMode = opMode;

    }

    public void init() {

        left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_back_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Log.i("IMUt", "starting imu stuff");

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        turnPID = new SynchronousPID(P_TURN_COEFF, I_TURN_COEFF, D_TURN_COEFF);
        drivePID = new SynchronousPID(P_DRIVE_COEFF, I_DRIVE_COEFF, D_DRIVE_COEFF);

        turnPID.setOutputRange(-TURN_SPEED, TURN_SPEED);
        turnPID.setInputRange(-180, 180);
    }


    //Stop All Motors
    private void stop_all_motors() {
        left_back_drive.setPower(0);
        right_back_drive.setPower(0);
        left_front_drive.setPower(0);
        right_front_drive.setPower(0);
    }


    /**
     * Function to move forwards, backwards, left, or right using PID.
     *
     * @param distance       Distance in inches to move. A positive distance makes the robot move
     *                       forwards and a negative distance makes the robot move backwards.
     *                       If strafe is enabled then forwards is right and backwards is left.
     * @param steeringToggle Steering to stay in a straight line. Just put true or false here.
     * @param strafe         Option to strafe instead of moving forwards/backwards. Put true or false here.
     */
    public void steeringDrive(double distance,
                              boolean steeringToggle,
                              boolean strafe) {

        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int strafeDirection = 0;

        if (distance > 0) {
            strafeDirection = 1;
        } else if (distance < 0) {
            strafeDirection = -1;
        }

        double steeringSpeed;
        double speed;

        drivePID.reset();
        turnPID.reset();

        if (!strafe) {
            int moveCounts = ((int) (distance * COUNTS_PER_INCH));
            int newFrontLeftTarget = left_front_drive.getCurrentPosition() + moveCounts;
            int newBackLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
            int newFrontRightTarget = right_front_drive.getCurrentPosition() + moveCounts;
            int newBackRightTarget = right_back_drive.getCurrentPosition() + moveCounts;
            int newAverageTarget;

            newAverageTarget = (newBackLeftTarget + newBackRightTarget + newFrontLeftTarget + newFrontRightTarget) / 4;

            int encAvg;
            ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            etime.reset();

            turnPID.setContinuous(true);
            gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            turnPID.setSetpoint(gyro_angle.firstAngle);
            turnPID.setOutputRange(-0.4, 0.4);

            drivePID.setContinuous(false);
            drivePID.setSetpoint(newAverageTarget);
            drivePID.setOutputRange(-0.4, 0.4);

            while (linearOpMode.opModeIsActive()) {
                encAvg = (left_front_drive.getCurrentPosition() + left_back_drive.getCurrentPosition() + right_back_drive.getCurrentPosition() + right_front_drive.getCurrentPosition()) / 4;

                if (((abs(newAverageTarget - encAvg)) < ENCODER_THRESHOLD)) {
                    break;
                }

                if (steeringToggle) {
                    gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    turnPID.calcInit();
                    steeringSpeed = turnPID.timedCalculate(gyro_angle.firstAngle);
                } else {
                    steeringSpeed = 0;
                }

                drivePID.calcInit();
                speed = drivePID.timedCalculate(encAvg);

                telemetry.addData("newAverageTarget", newAverageTarget);
                telemetry.addData("encAvg" , encAvg);
                telemetry.update();

                left_back_drive.setPower(speed - steeringSpeed);
                left_front_drive.setPower(speed - steeringSpeed);
                right_back_drive.setPower(speed + steeringSpeed);
                right_front_drive.setPower(speed + steeringSpeed);
            }
            stop_all_motors();
        }


        if (strafe) {
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            int newBackLeftTarget;
            int newFrontLeftTarget;
            int newBackRightTarget;
            int newFrontRightTarget;
            int newAverageTarget;

            if (strafeDirection == 1) {
                newBackLeftTarget = (left_back_drive.getCurrentPosition() - (moveCounts));
                newFrontLeftTarget = -(left_front_drive.getCurrentPosition() + (moveCounts));
                newBackRightTarget = -(right_back_drive.getCurrentPosition() + (moveCounts));
                newFrontRightTarget = (right_front_drive.getCurrentPosition() - (moveCounts));
                telemetry.addData("lrTarget", newBackLeftTarget);
                telemetry.addData("lfTarget", newFrontLeftTarget);
                telemetry.addData("rrTarget", newBackRightTarget);
                telemetry.addData("rfTarget", newFrontRightTarget);
                telemetry.update();
            } else {
                newBackLeftTarget = -(left_back_drive.getCurrentPosition() + (moveCounts));
                newFrontLeftTarget = (left_front_drive.getCurrentPosition() - (moveCounts));
                newBackRightTarget = (right_back_drive.getCurrentPosition() - (moveCounts));
                newFrontRightTarget = -(right_front_drive.getCurrentPosition() + (moveCounts));
                telemetry.addData("lrTarget", newBackLeftTarget);
                telemetry.addData("lfTarget", newFrontLeftTarget);
                telemetry.addData("rrTarget", newBackRightTarget);
                telemetry.addData("rfTarget", newFrontRightTarget);
                telemetry.update();
            }

            newAverageTarget = (abs(newBackLeftTarget) + abs(newBackRightTarget) + abs(newFrontLeftTarget) + abs(newFrontRightTarget)) / 4;

            int encAvg;
            ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            etime.reset();


            turnPID.setContinuous(true);
            gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            turnPID.setSetpoint(gyro_angle.firstAngle);
            turnPID.setOutputRange(-0.4, 0.4);


            drivePID.setContinuous(false);
            drivePID.setSetpoint(newAverageTarget);
            drivePID.setOutputRange(-0.4, 0.4);

            while (linearOpMode.opModeIsActive()) {

                encAvg = (abs(left_front_drive.getCurrentPosition()) + abs(left_back_drive.getCurrentPosition()) + abs(right_back_drive.getCurrentPosition()) + abs(right_front_drive.getCurrentPosition())) / 4;

                if (((abs(newAverageTarget - encAvg)) < ENCODER_THRESHOLD)) {
                    break;
                }

                if (steeringToggle) {
                    gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    steeringSpeed = turnPID.calculate(gyro_angle.firstAngle);
                } else {
                    steeringSpeed = 0;
                }


                drivePID.calcInit();
                speed = drivePID.timedCalculate(encAvg);

                if (strafeDirection == -1) {
                    left_back_drive.setPower(-strafeDirection * (speed - steeringSpeed));
                    left_front_drive.setPower(strafeDirection * (speed - steeringSpeed));
                    right_back_drive.setPower(strafeDirection * (speed + steeringSpeed));
                    right_front_drive.setPower(-strafeDirection * (speed + steeringSpeed));
                } else if (strafeDirection == 1) {
                    left_back_drive.setPower(-strafeDirection * (speed - steeringSpeed));
                    left_front_drive.setPower(strafeDirection * (speed - steeringSpeed));
                    right_back_drive.setPower(strafeDirection * (speed + steeringSpeed));
                    right_front_drive.setPower(-strafeDirection * (speed + steeringSpeed));
                }
            }
            stop_all_motors();
        }
    }

    public void gyroTurn(double speed, double angle) {
        turnPID.reset();
        turnPID.setContinuous(false);
        turnPID.setSetpoint(angle);
        turnPID.setOutputRange(-speed, speed);


        boolean doneTurning;

        ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        etime.reset();

        // keep looping while we are still active, and not on heading.


        left_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_front_drive.setPower(0);
        right_back_drive.setPower(0);

    }

    public void gyroHold(double speed, double angle, double holdTime) {

        turnPID.reset();
        turnPID.setSetpoint(angle);
        turnPID.setOutputRange(-speed, speed);
        turnPID.setDeadband(HEADING_THRESHOLD);

        ElapsedTime holdTimer = new ElapsedTime();


        // keep looping while we have time remaining.
        holdTimer.reset();
        while (linearOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(angle);
        }

        // Stop all motion;
        left_back_drive.setPower(0);
        left_front_drive.setPower(0);
        right_back_drive.setPower(0);
        right_front_drive.setPower(0);
    }

    /**
     * Turns using the drive PID instead of the turn PID.
     * @param angle Angle to turn.
     */
    public void arcTurn(double angle) {

        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double trackWidth = 14.0;//13.8;// TODO: Tune the trackwidth
        double radius = trackWidth / 2;
        double circumference = 2 * PI * radius;
        double distance = (angle / 180) * circumference;//Divide by 180 because we are using -180/+180 not 0/+360

        int direction = 0;
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        int newBackLeftTarget = 0 ;
        int newFrontLeftTarget = 0;
        int newBackRightTarget = 0;
        int newFrontRightTarget = 0;
//        int direction = (int) signum(distance);//Make sure this works.
        if (distance > 0) {
            direction = 1;
            newBackLeftTarget = (left_back_drive.getCurrentPosition() + (direction * moveCounts));
            newFrontLeftTarget = (left_front_drive.getCurrentPosition() + (direction * moveCounts));
            newBackRightTarget = (right_back_drive.getCurrentPosition() + (direction * moveCounts));
            newFrontRightTarget = (right_front_drive.getCurrentPosition() + (direction * moveCounts));
        } else if (distance < 0){
            direction = -1;
            newBackLeftTarget = (left_back_drive.getCurrentPosition() + (direction * moveCounts));
            newFrontLeftTarget = (left_front_drive.getCurrentPosition() + (direction * moveCounts));
            newBackRightTarget = (right_back_drive.getCurrentPosition() + (direction * moveCounts));
            newFrontRightTarget = (right_front_drive.getCurrentPosition() + (direction * moveCounts));
        }

        int newAverageTarget = (abs(newBackLeftTarget) + abs(newBackRightTarget) + abs(newFrontLeftTarget) + abs(newFrontRightTarget)) / 4;

        drivePID.reset();
        double speed;
        int encAvg;

        drivePID.setContinuous(false);
        drivePID.setSetpoint(newAverageTarget);
        drivePID.setOutputRange(-0.6, 0.6);

        while (linearOpMode.opModeIsActive()) {

//            telemetry.addData("Back Left Target", newBackLeftTarget);
//            telemetry.addData("Front Left Target", newFrontLeftTarget);
//            telemetry.addData("Back Right Target", newBackRightTarget);
//            telemetry.addData("Front Right Target", newFrontRightTarget);
            //No need for telemetry.update(); as setMotorPowers does it.

            encAvg = (abs(left_front_drive.getCurrentPosition()) + abs(left_back_drive.getCurrentPosition()) + abs(right_back_drive.getCurrentPosition()) + abs(right_front_drive.getCurrentPosition())) / 4;

            if (((abs(newAverageTarget - encAvg)) < ENCODER_THRESHOLD)) {
                break;
            }

            drivePID.calcInit();
            speed = drivePID.timedCalculate(encAvg);

            if (direction == 1)
            {
                left_front_drive.setPower(-speed);
                left_back_drive.setPower(-speed);
                right_front_drive.setPower(speed);
                right_back_drive.setPower(speed);
            }
            else if (direction == -1)
            {
                left_front_drive.setPower(speed);
                left_back_drive.setPower(speed);
                right_front_drive.setPower(-speed);
                right_back_drive.setPower(-speed);
            }


        }
        setMotorPowers(0,0,0,0);
    }

    private boolean onHeading(double angle) {

        double motorSpeed;


        while(linearOpMode.opModeIsActive()) {

            gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            turnPID.calcInit();
            motorSpeed = turnPID.timedCalculate(gyro_angle.firstAngle);

            if (turnPID.getError() < angle + HEADING_THRESHOLD) {
                break;
            }

            // Send desired speeds to motors.
            left_front_drive.setPower(motorSpeed);
            left_back_drive.setPower(motorSpeed);
            right_front_drive.setPower(-motorSpeed);
            right_back_drive.setPower(-motorSpeed);

            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/Angle", "%5.2f:%5.2f", turnPID.getError(), gyro_angle.firstAngle);
            telemetry.addData("Coef ", turnPID.getState());
            telemetry.addData("Speed.", "%5.2f:%5.2f", -motorSpeed, motorSpeed);
            telemetry.update();
        }
        return turnPID.onTarget(HEADING_THRESHOLD);
    }

    /**
     * Sets the specified powers to the motors.
     * @param leftFront The left front drive's speed
     * @param leftBack The left back drive's speed
     * @param rightFront The right front drive's speed
     * @param rightBack The right back drive's speed
     */
    public void setMotorPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
        left_front_drive.setPower(leftFront);
        left_back_drive.setPower(leftBack);
        right_front_drive.setPower(rightFront);
        right_back_drive.setPower(rightBack);
        telemetry.addData("leftFrontPower", leftFront);
        telemetry.addData("leftBackPower", leftBack);
        telemetry.addData("rightFrontPower", rightFront);
        telemetry.addData("rightBackPower", rightBack);
        telemetry.update();
    }
}