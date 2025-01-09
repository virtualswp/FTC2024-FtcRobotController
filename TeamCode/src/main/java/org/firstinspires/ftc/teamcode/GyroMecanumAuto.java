package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Gyro Mecanum Auto", group="Linear OpMode")
public class GyroMecanumAuto extends LinearOpMode {
    // Motors and sensors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;   // Has encoder
    private DcMotor rightRearDrive;  // Has encoder
    private IMU imu;

    // Constants for drive calculations
    private static final double TICKS_PER_REV = 537.7;  // For GoBILDA 5202/5203 motors
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // PID Constants
    private static final double P_DRIVE_GAIN = 0.03;
    private static final double P_STRAFE_GAIN = 0.015;
    private static final double P_TURN_GAIN = 0.025;

    // Drive speed constants
    private static final double DRIVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.4;
    private static final double STRAFE_SPEED = 0.3;

    //Freddy = FORWARD, Napoleon = BACKWARD
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_ORIENTATION = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    //<editor-fold desc="Enumerations">

    private enum strafeDirection{
        right,
        left,
        none
    }

    private enum armPosition {

        home,

        collectOut,

        collectUp,

        deliveryUp,

    }

    private enum slidePosition {
        home,

        lowBasket,

        highBasket

    }

    private enum driveMode {
        normal,
        collection,
        deposit
    }
    //</editor-fold>


    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        // Only reset encoders on rear motors
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set run modes
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                IMU_ORIENTATION)));

        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence
            gyroDrive(DRIVE_SPEED, 24.0, 0.0);     // Drive forward 24 inches
            //gyroStrafe(STRAFE_SPEED, 12.0, 0.0, strafeDirection.left);   // Strafe left 12 inches
            gyroStrafe(STRAFE_SPEED, 12.0, 0.0, strafeDirection.right);   // Strafe right 12 inches
            //gyroDrive(DRIVE_SPEED, -24.0, 0.0);     // Drive backwards 24 inches
            //gyroTurn(TURN_SPEED, 90.0);            // Turn left 90 degrees from original direction
            //gyroTurn(TURN_SPEED, -90.0);            // Turn right 90 degrees from original direction
            //gyroDrive(DRIVE_SPEED, 24.0, 90.0);    // Drive forward 24 inches maintaining 90 degrees
        }
    }

    public void gyroDrive(double speed, double distance, double angle) {
        int targetTicks = (int)(distance * TICKS_PER_INCH);
        resetRearEncoders();

        double direction = Math.signum(distance);

        while (opModeIsActive() && !isDriveComplete(targetTicks)) {
            double error = getAngleError(angle, getCurrentHeading());
            double correction = error * P_DRIVE_GAIN;

            double[] powers = calculateWheelPowers(speed * direction, 0.0, correction, strafeDirection.none);
            setMotorPowers(powers);

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Left Rear Pos", leftRearDrive.getCurrentPosition());
            telemetry.addData("Right Rear Pos", rightRearDrive.getCurrentPosition());
            telemetry.addData("Current Heading", getCurrentHeading());
            telemetry.update();
        }

        setMotorPowers(new double[]{0, 0, 0, 0});
    }

    public void gyroStrafe(double speed, double distance, double angle, strafeDirection direction) {
        int targetTicks = (int)(distance * TICKS_PER_INCH * 1.2); // 1.2x multiplier for strafing
        resetRearEncoders();

        while (opModeIsActive() && !isStrafeComplete(targetTicks)) {
            double error = getAngleError(angle, getCurrentHeading());
            double correction = error * P_STRAFE_GAIN;

            double[] powers = calculateWheelPowers(0.0, speed, correction, direction);
            setMotorPowers(powers);

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Left Rear Pos", leftRearDrive.getCurrentPosition());
            telemetry.addData("Right Rear Pos", rightRearDrive.getCurrentPosition());
            telemetry.addData("Current Heading", getCurrentHeading());
            telemetry.update();
        }

        setMotorPowers(new double[]{0, 0, 0, 0});
    }

    public void gyroTurn(double speed, double targetAngle) {
        double error;
        do {
            error = getAngleError(targetAngle, getCurrentHeading());
            double turnPower = error * P_TURN_GAIN;

            // Limit turn power to specified speed
            if (Math.abs(turnPower) > speed) {
                turnPower = Math.signum(turnPower) * speed;
            }

            double[] powers = calculateWheelPowers(0.0, 0.0, turnPower, strafeDirection.none);
            setMotorPowers(powers);

            telemetry.addData("Turning to", targetAngle);
            telemetry.addData("Current Angle", getCurrentHeading());
            telemetry.addData("Error", error);
            telemetry.update();

        } while (opModeIsActive() && Math.abs(error) > 2.0);

        setMotorPowers(new double[]{0, 0, 0, 0});
    }

    private double[] calculateWheelPowers(double drive, double strafe, double turn, strafeDirection strafeDirection) {
        double[] wheelPowers = new double[4];

        //Strafe Left
        if (strafeDirection == GyroMecanumAuto.strafeDirection.left){
            wheelPowers[0] = drive - strafe - turn;  // Left Front
            wheelPowers[1] = drive + strafe + turn;  // Right Front
            wheelPowers[2] = drive + strafe - turn;  // Left Rear
            wheelPowers[3] = drive - strafe + turn;  // Right Rear
        }
        else {
            //Strafe Right or none
            wheelPowers[0] = drive + strafe - turn;  // Left Front
            wheelPowers[1] = drive - strafe + turn;  // Right Front
            wheelPowers[2] = drive - strafe - turn;  // Left Rear
            wheelPowers[3] = drive + strafe + turn;  // Right Rear
        }

        // Normalize wheel powers
        double maxPower = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])),
                Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));

        if (maxPower > 1.0) {
            for (int i = 0; i < 4; i++) {
                wheelPowers[i] /= maxPower;
            }
        }

        return wheelPowers;
    }

    private void setMotorPowers(double[] powers) {
        leftFrontDrive.setPower(powers[0]);
        rightFrontDrive.setPower(powers[1]);
        leftRearDrive.setPower(powers[2]);
        rightRearDrive.setPower(powers[3]);
    }

    private void resetRearEncoders() {
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean isDriveComplete(int targetTicks) {
        // Average only the rear encoders
        int avgPosition = (Math.abs(leftRearDrive.getCurrentPosition()) +
                Math.abs(rightRearDrive.getCurrentPosition())) / 2;
        return Math.abs(avgPosition) >= Math.abs(targetTicks);
    }

    private boolean isStrafeComplete(int targetTicks) {
        // For strafing, check both rear encoders
        // Note: When strafing, rear wheels will rotate in opposite directions
        int leftPos = Math.abs(leftRearDrive.getCurrentPosition());
        int rightPos = Math.abs(rightRearDrive.getCurrentPosition());
        int avgPosition = (leftPos + rightPos) / 2;

        return avgPosition >= Math.abs(targetTicks);
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double angleError = targetAngle - currentAngle;

        while (angleError > 180) angleError -= 360;
        while (angleError <= -180) angleError += 360;
        return angleError;
    }

    private double getCurrentHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}