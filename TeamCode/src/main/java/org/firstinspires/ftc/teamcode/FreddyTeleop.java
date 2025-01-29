/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Device;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
 * Into The Deep Starter Robot
 * The code is structured as a LinearOpMode
 *
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */

@TeleOp(name = "Freddy Teleop", group = "Robot")
//@Disabled
public class FreddyTeleop extends LinearOpMode {

    //<editor-fold desc="Hardware Variables">
    public DcMotor leftFrontDriveMotor = null; //the left drivetrain motor
    public DcMotor rightFrontDriveMotor = null; //the right drivetrain motor
    public DcMotor leftRearDriveMotor = null; //the left drivetrain motor
    public DcMotor rightRearDriveMotor = null; //the right drivetrain motor

    public DcMotor armMotorLeft = null;

    public DcMotor armMotorRight = null;

    public DcMotor slideMotor = null;

    public DcMotor slideArmMotor = null;

    private Servo gripperWrist = null;

    private Servo gripperHand = null;

    private TouchSensor armButtonFront = null;                   // The REV Robotics touch sensor button on the arm rest tower (Detecting Down Position)

    private TouchSensor armButtonRear = null;                   // The REV Robotics touch sensor button on the upper electronics shelf (Detecting Up Position)

    private TouchSensor armSlideSwitch = null;                 // The REV Robotics Magnetic Limit Switch For the Slide Arm (Detecting Slide Extension Position)

    private TouchSensor armSlideButtonRear = null;              // The REV RObotics touch sensor button on the Swyft Slide back (Detecting Slide Back position)

    //</editor-fold>

    //<editor-fold desc="Member Variables">

    private armPosition currentArmPosition = armPosition.home;         //The current arm position

    private armPosition targetArmPosition = armPosition.home;          //The target arm position

    private slidePosition currentSlidePosition = slidePosition.home;            // The current slide position

    private slidePosition targetSlidePosition = slidePosition.home;             // The target slide position

    private driveMode currentDriveMode = driveMode.normal;              //The current drive mode

    private boolean isArmFrontButtonPressed = false;                     //Variable to determine if the front arm button is being pressed

    private boolean isArmBackButtonPressed = false;                     //Variable to determine if the back arm button is being pressed

    private boolean isArmSlideSwitchPressed = false;                    //Variable to determine if the slide's magnetic switch sensor is pressed

    private boolean isArmSlideBackButtonPressed = false;                //Variable to determine if the slide arm's back button / touch sensor is pressed

    private final ElapsedTime runtime = new ElapsedTime();

    //</editor-fold>

    //<editor-fold desc="Constants">

    private static final double ENCODER_ZERO_POWER = 0.1;               //The amount of power to have the motor use to brake (hold) the motor position.

    private static final int SLIDE_LOW_BASKET = 1500;                   //The degrees the encoder needs to move the slide motor to get to the low basket

    //freddy = 3500, Napoliean = 3500
    private int SLIDE_HIGH_BASKET = 3500;                  //The degrees the encoder needs to move the slide motor to get to the high basket

    //freddy = 3550, napoleon = 3400
    private int SLIDE_HIGH_BASKET_MIN = 3400;              //The minimum height the magnetic switch sensor can be valid at

    //Napoleon = 450, Freddy = 750
    private int SLIDE_COLLECT_OUT = 450;                   //The degrees the encoder needs to move the slide motor to get to the collect out position

    private static final int SLIDE_COLLECT_OUT_SPEED = 1200;             //The velocity to move the slide out to collect out.

    private static final int SLIDE_COLLECT_UP_SPEED = 900;              //The velocity to move the slide in to the collect up position.

    private static final int SLIDE_HOME_RESET = -100;                   // The position to move the slide motor past the home (0) position to account for any variance with the encoder

    private static final int ARM_COLLECT_DELIVERY = -3700;             //The degrees to move the arm straight up to the delivery up position

    private static final int ARM_HOME_RESET = 400;                     // The position to move the arm motor past the home (0) position to account for any variance with the encoder

    private static final double HAND_OPEN_POSITION = 0.0;               // The servo position for the hand to be fully open.

    //Napoleon = 0.57, Freddy = 0.64
    private double HAND_CLOSED_POSITION = 0.57;            // The servo position for the hand to be fully closed.

    // Napoleon = 0.70, Freddy = 0.67
    private double WRIST_DOWN_POSITION = 0.70;             // The servo position for the wrist to be fully down.

    private static final double WRIST_BACK_POSITION = 0.0;              // The servo position for the wrist to be fully back.

    private static final double WRIST_DELIVERY_POSITION = 0.30;         // The servo position for the wrist when delivering to the baskets

    private static final double WRIST_COLLECT_UP_POSITION = 0.5;        // The servo position for the wrist when moving to collect up position (slightly back to go over the bar)

    private static final robot currentRobot = robot.Freddy;                   // The robot

    //</editor-fold>

    //<editor-fold desc="Enumerations">

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
        fast,
        normal,
        slow
    }

    private enum robot {
        Freddy,

        Napoleon,
    }

    //</editor-fold>

    //<editor-fold desc="Op Mode & Handlers">

    @Override
    public void runOpMode() {
        //Configure the hardware and hardware values
        this.ConfigureHardware();
        this.ConfigureHardwareValues();

        /* Wait for the game driver to press play */
        waitForStart();

        // Check if the arm is starting from home or collect down //
        this.HandleArmSensors();
        this.CheckInitialArmState();
        this.SetHardwareDefaultPositions();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            this.HandleTeleopDrive();
            this.HandleTeleopWrist();
            this.HandleTeleopHand();
            this.HandleTeleopArm();
            this.HandleTeleopAscent();
            this.HandleArmSensors();

            this.HandleTeleopTelemetry();
        }
    }

    private void HandleTeleopAscent() {
        final double rightArmPower = 0.8;
        final double leftArmPower = 0.8;

        //Left Lift Arm
        if (gamepad1.dpad_up) {
            this.armMotorLeft.setPower(-leftArmPower);
        } else if (gamepad1.dpad_down) {
            this.armMotorLeft.setPower(leftArmPower);
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down){
            this.armMotorLeft.setPower(0.0);
        }

        //Right lift arm
        if (gamepad1.y) {
            this.armMotorRight.setPower(-rightArmPower);
        } else if (gamepad1.a) {
            this.armMotorRight.setPower(rightArmPower);
        } else if (!gamepad1.y && !gamepad1.a){
            this.armMotorRight.setPower(0.0);
        }

        //End Ascent
        if (gamepad1.x){
            //Hold
            runtime.reset();

            //Run for 10 seconds to hang
            while (opModeIsActive() && runtime.seconds() < 10) {

                //Check for the kill switch on game pad 1 or 2
                if ((gamepad1.b) || (gamepad2.x)){
                    break;
                }

                //Otherwise hold the power on the motors
                this.armMotorLeft.setPower(leftArmPower);
                this.armMotorRight.setPower(rightArmPower);
            }
        }
    }

    private void HandleTeleopArm() {
        //--------------Arm-----------------------------
        //NOTE: The arm should only move if the slide is all the way down in the home position.
        if (this.currentSlidePosition == slidePosition.home) {
            if (gamepad2.dpad_down) {
                //Move to collect down position
                this.targetArmPosition = armPosition.home;
            } else if (gamepad2.dpad_left) {
                //Move to collect out position
                this.targetArmPosition = armPosition.collectOut;
            } else if (gamepad2.dpad_right) {
                //Move to collect up position
                this.targetArmPosition = armPosition.collectUp;
            } else if (gamepad2.dpad_up) {
                //Move to the delivery (arm straight up) position
                this.targetArmPosition = armPosition.deliveryUp;
            }

            //Next, check if we need to move the arm
            if (!this.isArmAtTargetPosition()) {
                switch (this.targetArmPosition) {
                    case collectOut:
                        this.MoveArmToCollectOutPosition();
                        break;
                    case collectUp:
                        this.MoveArmToCollectUpPosition();
                        break;
                    case home:
                        this.MoveArmToHomePosition();
                        break;
                    case deliveryUp:
                        this.MoveArmToDeliveryUpPosition();
                        break;
                }
            }
        }

        //------------Slide-----------------------------
        //NOTE: Slide motion only should happen when the arm is in the delivery up position - this is for safety
        // of the robot during teleop.
        if (this.currentArmPosition == armPosition.deliveryUp) {
            if (gamepad2.y) {
                //Move to high basket
                this.targetSlidePosition = slidePosition.highBasket;
            } else if (gamepad2.x) {
                // Move to the low basket
                this.targetSlidePosition = slidePosition.lowBasket;
            } else if (gamepad2.a) {
                //Move to the home position
                this.targetSlidePosition = slidePosition.home;
            }
        }

        //Next, check if we need to move the slide
        if (!this.isSlideAtTargetPosition()) {
            switch (this.targetSlidePosition) {
                case home:
                    this.MoveSlideToHomePosition();
                    break;
                case lowBasket:
                    this.MoveSlideToLowBasketPosition();
                    break;
                case highBasket:
                    this.MoveSlideToHighBasketPosition();
                    break;
            }
        }
    }

    private void HandleArmSensors() {
        this.isArmFrontButtonPressed = armButtonFront.isPressed();
        this.isArmBackButtonPressed = armButtonRear.isPressed();
        this.isArmSlideSwitchPressed = armSlideSwitch.isPressed();
        this.isArmSlideBackButtonPressed = armSlideButtonRear.isPressed();
    }

    private void HandleTeleopDrive() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0.0;
        double rightPower = 0.0;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafeRight = gamepad1.right_trigger;
        double strafeLeft = gamepad1.left_trigger;

        boolean rightUpStrafe = gamepad1.dpad_up;
        boolean rightDownStrafe = gamepad1.dpad_right;
        boolean leftUpStrafe = gamepad1.dpad_left;
        boolean leftDownStrafe = gamepad1.dpad_down;

        //Speed settings
        this.currentDriveMode = driveMode.normal;

        if (gamepad1.left_bumper)
        {
            this.currentDriveMode = driveMode.slow;
        }
        else if (gamepad1.right_bumper)
        {
            this.currentDriveMode = driveMode.fast;
        }


        if (strafeLeft > 0) {
            ChassisMotorValues c = new ChassisMotorValues();
            c = this.strafeLeft(strafeLeft);

            if (this.currentDriveMode == driveMode.normal) {
                // Normal Drive Mode
                float collectorModeFactor = 0.7F;
                leftRearDriveMotor.setPower(c.leftRear * collectorModeFactor);
                leftFrontDriveMotor.setPower(c.leftFront * collectorModeFactor);
                rightRearDriveMotor.setPower(c.rightRear * collectorModeFactor);
                rightFrontDriveMotor.setPower(c.rightFront * collectorModeFactor);
            } else if (this.currentDriveMode == driveMode.slow) {
                // Slow Drive Mode
                float collectorModeFactor = 0.3F;
                leftRearDriveMotor.setPower(c.leftRear * collectorModeFactor);
                leftFrontDriveMotor.setPower(c.leftFront * collectorModeFactor);
                rightRearDriveMotor.setPower(c.rightRear * collectorModeFactor);
                rightFrontDriveMotor.setPower(c.rightFront * collectorModeFactor);
            } else {
                // Fast Drive mode
                leftRearDriveMotor.setPower(c.leftRear);
                leftFrontDriveMotor.setPower(c.leftFront);
                rightRearDriveMotor.setPower(c.rightRear);
                rightFrontDriveMotor.setPower(c.rightFront);
            }
        } else if (strafeRight > 0) {
            ChassisMotorValues c = new ChassisMotorValues();
            c = this.strafeRight(strafeRight);

            if (this.currentDriveMode == driveMode.normal) {
                // Collect drive mode (slow down)
                float collectorModeFactor = 0.7F;
                leftRearDriveMotor.setPower(c.leftRear * collectorModeFactor);
                leftFrontDriveMotor.setPower(c.leftFront * collectorModeFactor);
                rightRearDriveMotor.setPower(c.rightRear * collectorModeFactor);
                rightFrontDriveMotor.setPower(c.rightFront * collectorModeFactor);
            } else if (this.currentDriveMode == driveMode.slow) {
                // Deposit drive mode (really slow down)
                float collectorModeFactor = 0.3F;
                leftRearDriveMotor.setPower(c.leftRear * collectorModeFactor);
                leftFrontDriveMotor.setPower(c.leftFront * collectorModeFactor);
                rightRearDriveMotor.setPower(c.rightRear * collectorModeFactor);
                rightFrontDriveMotor.setPower(c.rightFront * collectorModeFactor);
            } else {
                // Normal Drive mode (full speed)
                leftRearDriveMotor.setPower(c.leftRear);
                leftFrontDriveMotor.setPower(c.leftFront);
                rightRearDriveMotor.setPower(c.rightRear);
                rightFrontDriveMotor.setPower(c.rightFront);
            }
//        } else if (gamepad1.left_bumper) {
//            diagonalStrafe(rightUpStrafe, rightDownStrafe, leftUpStrafe, leftDownStrafe);
        } else {
            // Tank Mode uses one stick to control each wheel.
            if (this.currentDriveMode == driveMode.normal) {
                // Collect drive mode (slow down)
                float collectorModeFactor = 0.7F;
                leftPower = gamepad1.left_stick_y * collectorModeFactor;
                rightPower = gamepad1.right_stick_y * collectorModeFactor;
            } else if (this.currentDriveMode == driveMode.slow) {
                // Deposit drive mode (really slow down)
                float collectorModeFactor = 0.4F;
                leftPower = gamepad1.left_stick_y * collectorModeFactor;
                rightPower = gamepad1.right_stick_y * collectorModeFactor;
            } else {
                // Normal Drive mode (full speed)
                leftPower = gamepad1.left_stick_y;
                rightPower = gamepad1.right_stick_y;
            }


            leftRearDriveMotor.setPower(leftPower);
            leftFrontDriveMotor.setPower(leftPower);
            rightRearDriveMotor.setPower(-rightPower);
            rightFrontDriveMotor.setPower(-rightPower);
        }
    }

    private void HandleTeleopWrist() {
        //First, define the values of the servo positions
        final double SERVO_MIN_POS = WRIST_DOWN_POSITION;       //The servo min position (Gripper Closed)
        final double SERVO_MAX_POS = WRIST_BACK_POSITION;       //The servo max position (Gripper Opened)
        final double SERVO_MOVE_SPEED = 0.01;                   //The number of ticks to move by

        //Next, check if the left trigger is being pushed, if so, tip the wrist down.
        if (gamepad2.left_bumper) {
            //We don't want to exceed the position the wrist can move, so calculate the minimum current position.
            double newPosition = gripperWrist.getPosition() - SERVO_MOVE_SPEED;

            if (newPosition >= SERVO_MAX_POS) {
                //Set the servo new position
                gripperWrist.setPosition(newPosition);
            }
        }

        //Next, check if the right bumper is being pushed, if so, tip the wrist backwards.
        if (gamepad2.right_bumper) {
            //First calculate the target new position
            double newPosition = gripperWrist.getPosition() + SERVO_MOVE_SPEED;

            // Next, check to make sure the new position doesn't exceed the minimum position
            if (newPosition <= SERVO_MIN_POS) {
                //Set the servo new position
                gripperWrist.setPosition(newPosition);
            }
        }
    }

    private void HandleTeleopHand() {
        //First, define the values of the servo positions
        final double SERVO_MOVE_SPEED = 0.03;               //The number of ticks to move by

        //Next, check if the left trigger is being pushed, if so, open the gripper.
        if (gamepad2.left_trigger > 0) {
            //First calculate the target new position
            double newPosition = gripperHand.getPosition() - SERVO_MOVE_SPEED;

            // Next, check to make sure the new position doesn't exceed the minimum position
            if (newPosition >= HAND_OPEN_POSITION) {
                //Set the servo new position
                gripperHand.setPosition(newPosition);
            }

            //Set the servo new position
            gripperHand.setPosition(newPosition);
        }

        //Next, check if the right trigger is being pushed, if so, close the gripper.
        if (gamepad2.right_trigger > 0) {
            //First calculate the target new position
            double newPosition = gripperHand.getPosition() + SERVO_MOVE_SPEED;

            // Next, check to make sure the new position doesn't exceed the minimum position
            if (newPosition <= HAND_CLOSED_POSITION) {
                //Set the servo new position
                  gripperHand.setPosition(newPosition);
            }
        }

    }

    private void HandleTeleopTelemetry() {
        /* Check to see if our arm and slider is over the current limit, and report via telemetry. */
        if (((DcMotorEx) armMotorLeft).isOverCurrent()) {
            telemetry.addLine("LEFT ARM MOTOR EXCEEDED CURRENT LIMIT!");
        }

        if (((DcMotorEx) armMotorRight).isOverCurrent()) {
            telemetry.addLine("RIGHT ARM MOTOR EXCEEDED CURRENT LIMIT!");
        }

        if (((DcMotorEx) slideMotor).isOverCurrent()) {
            telemetry.addLine("SLIDE MOTOR EXCEEDED CURRENT LIMIT!");
        }


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("Arm Position", this.currentArmPosition);

        // Telemetry sometimes useful for debugging
        int currentSlideArmMotorPosition = this.slideArmMotor.getCurrentPosition();
        telemetry.addData("currentSlideArmMotorPosition", currentSlideArmMotorPosition);

        int currentSlidePosition = this.slideMotor.getCurrentPosition();
        telemetry.addData("currentSlidePosition: ", currentSlidePosition);

        double currentWristPosition = this.gripperWrist.getPosition();
        telemetry.addData("currentWristPosition: ", currentWristPosition);

        double currentHandPosition = this.gripperHand.getPosition();
        telemetry.addData("currentHandPosition", currentHandPosition);


        //telemetry.addData("currentMotorPosition: ", currentMotorPosition);
        //telemetry.addData("targetMotorPosition: ", targetMotorPosition);

        telemetry.update();
    }

    //</editor-fold>

    //<editor-fold desc="Arm Up / Down Methods">

    private void MoveArmToHomePosition() {
        //First, determine the current position
        switch (this.currentArmPosition) {
            case collectUp:
                // Move the arm down to the down up position using the encoder
                this.slideArmMotor.setTargetPosition(ARM_HOME_RESET);
                ((DcMotorEx) this.slideArmMotor).setVelocity(SLIDE_COLLECT_UP_SPEED);
                this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position (Either from the encoder or the button press)
                if (this.isMotorAtPosition(slideArmMotor) || this.isArmFrontButtonPressed) {
                    //Set the encoder position to the current position to hold the arm up
                    this.slideArmMotor.setTargetPosition(this.slideArmMotor.getCurrentPosition());
                    this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    //Success
                    this.currentArmPosition = armPosition.home;
                }

                break;
            case collectOut:
                // Move the slide in the home position using the encoder until the button is pressed
                this.slideMotor.setTargetPosition(SLIDE_HOME_RESET);
                ((DcMotorEx) this.slideMotor).setVelocity(SLIDE_COLLECT_UP_SPEED);
                this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position
                if (this.isMotorAtPosition(slideMotor) || this.isArmSlideBackButtonPressed) {
                    //If the back button is being pressed, reset the encoder to 0
                    if (this.isArmSlideBackButtonPressed)
                    {
                        //Set the encoder position to the current position to hold the arm up
                        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slideMotor.setTargetPosition(0);
                        slideMotor.setPower(ENCODER_ZERO_POWER);
                        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    else {
                        //Set the encoder position to the current position to hold the arm up
                        this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    //Set the encoder position to the current position to hold the arm up
                    this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                    this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Success
                    this.currentArmPosition = armPosition.home;
                }

                break;
            case deliveryUp:
                // Move the arm down to the down up position using the encoder
                this.slideArmMotor.setTargetPosition(ARM_HOME_RESET);

                // Next, determine where the arm is at so we can throttle the speed of the arm movement
                int degreesUntilTarget = Math.abs(this.slideArmMotor.getTargetPosition() - this.slideArmMotor.getCurrentPosition());

                if (degreesUntilTarget <= 500) {
                    // Run at a slow speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(300);
                } else if (degreesUntilTarget <= 1000) {
                    // Run at a medium speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(500);
                } else {
                    //Run at full speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(3000);
                }

                this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position (Either from the encoder or the button press)
                if (this.isMotorAtPosition(slideArmMotor) || this.isArmFrontButtonPressed) {
                    //Set the encoder position to the current position to hold the arm up
                    this.slideArmMotor.setTargetPosition(this.slideArmMotor.getCurrentPosition());
                    this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Success
                    this.currentArmPosition = armPosition.home;
                }

                break;

            case home:
                //Already here, shouldn't happen
                break;
        }
    }

    private void MoveArmToCollectOutPosition() {
        // First, check the current position
        switch (this.currentArmPosition) {
            case home:
                //First move the wrist down and open the gripper
                this.gripperWrist.setPosition(WRIST_DOWN_POSITION);
                this.gripperHand.setPosition(HAND_OPEN_POSITION);

                // Move the slide out the collect up position using the encoder
                this.slideMotor.setTargetPosition(SLIDE_COLLECT_OUT);
                ((DcMotorEx) this.slideMotor).setVelocity(SLIDE_COLLECT_OUT_SPEED);
                this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position
                if (this.isMotorAtPosition(slideMotor)) {
                    //Success
                    this.currentArmPosition = armPosition.collectOut;
                }

                break;
            case collectOut:
                //This shouldn't happen, it's already in collect up position
                break;
            case collectUp:
                //First move the wrist down and open the gripper
                this.gripperWrist.setPosition(WRIST_DOWN_POSITION);
                this.gripperHand.setPosition(HAND_OPEN_POSITION);

                // Move the slide out the collect out position using the encoder
                this.slideMotor.setTargetPosition(SLIDE_COLLECT_OUT);
                ((DcMotorEx) this.slideMotor).setVelocity(SLIDE_COLLECT_OUT_SPEED);
                this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position
                if (this.isMotorAtPosition(slideMotor)) {
                    //Success
                    this.currentArmPosition = armPosition.collectOut;
                }

                break;
            case deliveryUp:
                //First move the wrist down and open the gripper
                this.gripperWrist.setPosition(WRIST_DOWN_POSITION);
                this.gripperHand.setPosition(HAND_OPEN_POSITION);

                // Move the arm up to the collect up position using the encoder
                this.slideArmMotor.setTargetPosition(ARM_HOME_RESET);

                // Next, determine where the arm is at so we can throttle the speed of the arm movement
                int degreesUntilTarget = Math.abs(this.slideArmMotor.getTargetPosition() - this.slideArmMotor.getCurrentPosition());

//                if (degreesUntilTarget <= 1000) {
//                    // Run at a slow speed
//                    ((DcMotorEx) this.slideArmMotor).setVelocity(300);
//                } else if (degreesUntilTarget <= 1500) {
//                    // Run at a medium speed
//                    ((DcMotorEx) this.slideArmMotor).setVelocity(500);
//                } else {
//                    //Run at full speed
//                    ((DcMotorEx) this.slideArmMotor).setVelocity(3000);
//                }

                if (degreesUntilTarget <= 1500) {
                    // Run at a medium speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(500);
                } else {
                    //Run at full speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(3000);
                }

                this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position
                if (this.isMotorAtPosition(slideArmMotor) || this.isArmFrontButtonPressed) {
                    //Set the encoder position to the current position to hold the arm up
                    this.slideArmMotor.setTargetPosition(this.slideArmMotor.getCurrentPosition());
                    this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Move the slide out the collect up position using the encoder
                    this.slideMotor.setTargetPosition(SLIDE_COLLECT_OUT);
                    ((DcMotorEx) this.slideMotor).setVelocity(SLIDE_COLLECT_OUT_SPEED);
                    this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Next, check if the arm is at the final position
                    if (this.isMotorAtPosition(slideMotor)) {
                        //Success
                        this.currentArmPosition = armPosition.collectOut;
                    }
                }

                break;
        }
    }

    private void MoveArmToCollectUpPosition() {
        //First, determine the current position
        switch (this.currentArmPosition) {
            case collectUp:
                //Already in position, this shouldn't happen.
                break;
            case collectOut:
                //First move the wrist up
                this.gripperWrist.setPosition(WRIST_COLLECT_UP_POSITION);

                // Move the slide in the home position using the encoder until the button is pressed
                this.slideMotor.setTargetPosition(-20);
                ((DcMotorEx) this.slideMotor).setVelocity(SLIDE_COLLECT_UP_SPEED);
                this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position
                if (this.isMotorAtPosition(slideMotor) || this.isArmSlideBackButtonPressed) {
                    //If the back button is being pressed, reset the encoder to 0
                    if (this.isArmSlideBackButtonPressed)
                    {
                        //Set the encoder position to the current position to hold the arm up
                        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slideMotor.setTargetPosition(0);
                        slideMotor.setPower(ENCODER_ZERO_POWER);
                        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    else {
                        //Set the encoder position to the current position to hold the arm up
                        this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    //Success
                    this.currentArmPosition = armPosition.collectUp;
                }

                break;
            case home:
                //First move the wrist up
                this.gripperWrist.setPosition(WRIST_COLLECT_UP_POSITION);

                //Success
                this.currentArmPosition = armPosition.collectUp;

                break;
            case deliveryUp:
                //First move the wrist up
                this.gripperWrist.setPosition(WRIST_COLLECT_UP_POSITION);

                //Next, move the arm home
                this.slideArmMotor.setTargetPosition(ARM_HOME_RESET);

                // Next, determine where the arm is at so we can throttle the speed of the arm movement
                int degreesUntilTarget = Math.abs(this.slideArmMotor.getTargetPosition() - this.slideArmMotor.getCurrentPosition());

                if (degreesUntilTarget <= 500) {
                    // Run at a slow speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(300);
                } else if (degreesUntilTarget <= 1000) {
                    // Run at a medium speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(500);
                } else {
                    //Run at full speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(3000);
                }

                this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position
                if (this.isMotorAtPosition(slideArmMotor) || this.isArmFrontButtonPressed) {
                    //Set the encoder position to the current position to hold the arm up
                    this.slideArmMotor.setTargetPosition(this.slideArmMotor.getCurrentPosition());
                    this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Success
                    this.currentArmPosition = armPosition.collectUp;
                }

                break;
        }
    }

    private void MoveArmToDeliveryUpPosition() {
        // First, check the current position
        switch (this.currentArmPosition) {
            case home:
                //NOTE Same code to go to Delivery Up from Collect Up and Home, don't add a break statement
            case collectUp: {
                //Move the wrist to the delivery position
                this.gripperWrist.setPosition(WRIST_DELIVERY_POSITION);

                // Move the arm up to the delivery up position using the encoder
                this.slideArmMotor.setTargetPosition(ARM_COLLECT_DELIVERY);

                // Next, determine where the arm is at so we can throttle the speed of the arm movement
                int degreesUntilTarget = Math.abs(this.slideArmMotor.getTargetPosition() - this.slideArmMotor.getCurrentPosition());

                if (degreesUntilTarget <= 500) {
                    // Run at a slow speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(350);
//                } else if (degreesUntilTarget <= 1000) {
//                    // Run at a medium speed
//                    ((DcMotorEx) this.slideArmMotor).setVelocity(650);
                } else {
                    //Run at full speed
                    ((DcMotorEx) this.slideArmMotor).setVelocity(3000);
                }

                this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position (Either from the encoder or the button press)
                if (this.isMotorAtPosition(slideArmMotor) || this.isArmBackButtonPressed) {
                    //Set the encoder position to the current position to hold the arm up
                    this.slideArmMotor.setTargetPosition(this.slideArmMotor.getCurrentPosition());
                    this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Success
                    this.currentArmPosition = armPosition.deliveryUp;
                }
            }
            break;

            case collectOut: {
                //Move the wrist to the delivery position
                this.gripperWrist.setPosition(WRIST_DELIVERY_POSITION);

                //Move the slide in using the encoder until the back slide button is pressed
                this.slideMotor.setTargetPosition(SLIDE_HOME_RESET);
                ((DcMotorEx) this.slideMotor).setVelocity(300);
                this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Next, check if the arm is at the final position
                if (this.isMotorAtPosition(slideMotor) || this.isArmSlideBackButtonPressed) {
                    //Set the encoder position to the current position to reset the slide to 0
                    this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                    this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Move the arm up to the delivery up position using the encoder
                    this.slideArmMotor.setTargetPosition(ARM_COLLECT_DELIVERY);

                    // Next, determine where the arm is at so we can throttle the speed of the arm movement
                    int degreesUntilTarget = Math.abs(this.slideArmMotor.getTargetPosition() - this.slideArmMotor.getCurrentPosition());

                    if (degreesUntilTarget <= 500) {
                        // Run at a slow speed
                        ((DcMotorEx) this.slideArmMotor).setVelocity(350);
                    } else if (degreesUntilTarget <= 1000) {
                        // Run at a medium speed
                        ((DcMotorEx) this.slideArmMotor).setVelocity(650);
                    } else {
                        //Run at full speed
                        ((DcMotorEx) this.slideArmMotor).setVelocity(3000);
                    }

                    this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Next, check if the arm is at the final position (Either from the encoder or the button press)
                    if (this.isMotorAtPosition(slideArmMotor) || this.isArmBackButtonPressed) {
                        //Set the encoder position to the current position to hold the arm up
                        this.slideArmMotor.setTargetPosition(this.slideArmMotor.getCurrentPosition());
                        this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        //Success
                        this.currentArmPosition = armPosition.deliveryUp;
                    }
                }
            }
            break;

            case deliveryUp:
                //This shouldn't happen, it's already in delivery up position
                break;
        }
    }

    //</editor-fold>

    //<editor-fold desc="Basket Methods">

    private void MoveSlideToHomePosition() {
        //Move the wrist to the delivery position
        this.gripperWrist.setPosition(WRIST_DOWN_POSITION);

        slideMotor.setTargetPosition(0);
        ((DcMotorEx) slideMotor).setVelocity(1500);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (this.isMotorAtPosition(slideMotor)) {
            this.currentSlidePosition = slidePosition.home;
        }
    }

    private void MoveSlideToHighBasketPosition() {
        slideMotor.setTargetPosition(SLIDE_HIGH_BASKET);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Next, determine where the side is at so we can throttle the speed of the slide movement
        int degreesUntilTarget = Math.abs(this.slideMotor.getTargetPosition() - this.slideMotor.getCurrentPosition());

        if (degreesUntilTarget <= 500) {
            // Run at a slow speed
            ((DcMotorEx) this.slideMotor).setVelocity(350);
//        } else if (degreesUntilTarget <= 1000) {
//            // Run at a medium speed
//            ((DcMotorEx) this.slideMotor).setVelocity(650);
        } else {
            //Run at full speed
            ((DcMotorEx) this.slideMotor).setVelocity(2000);
        }

        if (this.isMotorAtPosition(slideMotor) ||
                (this.isArmSlideSwitchPressed && (Math.abs(slideMotor.getCurrentPosition()) >= SLIDE_HIGH_BASKET_MIN)))
        {
                //Set the encoder position to the current position to hold the slide up
                this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.currentSlidePosition = slidePosition.highBasket;
        }
    }

    private void MoveSlideToLowBasketPosition() {
        //First determine the current

        slideMotor.setTargetPosition(SLIDE_LOW_BASKET);
        ((DcMotorEx) slideMotor).setVelocity(1500);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (this.isMotorAtPosition(slideMotor)) {
            this.currentSlidePosition = slidePosition.lowBasket;
        }
    }

    //</editor-fold>

    //<editor-fold desc="Drive / Strafe Methods">

    public ChassisMotorValues strafeRight(double strafePower) {
        ChassisMotorValues result = new ChassisMotorValues();

        result.leftFront = -strafePower;    //Should move forwards
        result.rightFront = -strafePower;    //Should move backwards
        result.leftRear = strafePower;     //Should move backwards
        result.rightRear = strafePower;     //Should move forwards

        return result;
    }

    public ChassisMotorValues strafeLeft(double strafePower) {
        ChassisMotorValues result = new ChassisMotorValues();

        result.leftFront = strafePower;    //Should move backwards
        result.rightFront = strafePower;    //Should move forwards
        result.leftRear = -strafePower;     //Should move forwards
        result.rightRear = -strafePower;     //Should move backwards

        return result;
    }

    public void diagonalStrafe(boolean rightUpStrafe, boolean rightDownStrafe, boolean leftUpStrafe, boolean leftDownStrafe) {
        if (rightUpStrafe == true) {
            leftFrontDriveMotor.setPower(0.45);
            rightRearDriveMotor.setPower(-0.45);
        } else if (rightDownStrafe == true) {
            rightFrontDriveMotor.setPower(-0.45);
            leftRearDriveMotor.setPower(0.45);
        } else if (leftUpStrafe == true) {
            rightFrontDriveMotor.setPower(0.45);
            leftRearDriveMotor.setPower(-0.45);
        } else if (leftDownStrafe == true) {
            leftFrontDriveMotor.setPower(-0.45);
            rightRearDriveMotor.setPower(0.45);
        } else {
            leftFrontDriveMotor.setPower(0);
            rightRearDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftRearDriveMotor.setPower(0);
        }
    }
    //</editor-fold>

    //<editor-fold desc="Utility Methods">

    private void CheckInitialArmState() {
        /* This method will check where the arm is starting at (from the prior auto code or teleop) */
        if (this.isArmFrontButtonPressed) {
            this.currentArmPosition = armPosition.home;
        }
    }

    private void SetHardwareDefaultPositions() {
        /* This method will set any hardware default positions */

        //Set the hand gripper to an open position to start
        //this.gripperHand.setPosition(HAND_OPEN_POSITION);                  //0.0 = All the way open, 1.0 is all the way closed.
        this.gripperWrist.setPosition(WRIST_BACK_POSITION);                //0.0 = All the way down, 1.0 is all the way back.
    }

    private boolean isArmAtTargetPosition() {
        return (this.currentArmPosition == this.targetArmPosition);
    }

    private boolean isSlideAtTargetPosition() {
        return (this.currentSlidePosition == this.targetSlidePosition);
    }

    public boolean isMotorAtPosition(DcMotor motor) {
        if (!motor.isBusy()) {
            telemetry.addData("motor: ", motor.isBusy());
            return true;
        }

        int varianceFactor = 10;

        int currentMotorPosition = motor.getCurrentPosition();
        int targetMotorPosition = motor.getTargetPosition();

        int error = Math.abs(targetMotorPosition - currentMotorPosition);

        return error <= varianceFactor;
    }

    private void ConfigureHardwareValues(){
        if (currentRobot == robot.Freddy) {
            SLIDE_HIGH_BASKET = 3450;
            SLIDE_HIGH_BASKET_MIN = 3300;
            SLIDE_COLLECT_OUT = 550;
            HAND_CLOSED_POSITION = 0.67;
            WRIST_DOWN_POSITION = 0.67;
        }
        else if (currentRobot == robot.Napoleon){
            SLIDE_HIGH_BASKET = 3450;
            SLIDE_HIGH_BASKET_MIN = 3300;
            SLIDE_COLLECT_OUT = 450;
            HAND_CLOSED_POSITION = 0.57;
            WRIST_DOWN_POSITION = 0.70;
        }
    }

    private void ConfigureHardware() {
        /* Define and Initialize Motors */
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDrive");          //Control Hub Motor Port 0
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDrive");        //Control Hub Motor Port 1
        leftRearDriveMotor = hardwareMap.get(DcMotor.class, "leftRearDrive");            //Control Hub Motor Port 2
        rightRearDriveMotor = hardwareMap.get(DcMotor.class, "rightRearDrive");          //Control Hub Motor Port 3

        gripperWrist = hardwareMap.get(Servo.class, "gripperWrist");                     //Expansion Hub Servo Port 0
        gripperHand = hardwareMap.get(Servo.class, "gripperHand");                       //Expansion Hub Servo Port 1

        armMotorLeft = hardwareMap.get(DcMotor.class, "armleft");                        //Expansion Hub Motor Port 2
        armMotorRight = hardwareMap.get(DcMotor.class, "armright");                      //Expansion Hub Motor Port 3
        slideMotor = hardwareMap.get(DcMotor.class, "slide");                            //Expansion Hub Motor Port 0
        slideArmMotor = hardwareMap.get(DcMotor.class, "slideArm");                      //Expansion Hub Motor Port 1

        armButtonFront = hardwareMap.get(TouchSensor.class, "armFrontButton");           //Expansion Hub Sensor Port 0-1
        armButtonRear = hardwareMap.get(TouchSensor.class, "armBackButton");             //Expansion Hub Sensor Port 2-3
        armSlideSwitch = hardwareMap.get(TouchSensor.class, "armSlideSwitch");           //Expansion Hub Sensor Port 4-5
        armSlideButtonRear = hardwareMap.get(TouchSensor.class, "armSlideBackButton");   //Control Hub Sensor Port 0-1

        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideArmMotor.setDirection(DcMotor.Direction.FORWARD);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotorLeft).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) armMotorRight).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) slideMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) slideArmMotor).setCurrentAlert(5, CurrentUnit.AMPS);


        //Set the arm motor's motor encoder to 0
//        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set the arm motor to use the encoder
        slideArmMotor.setTargetPosition(0);
        slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the slide motor to use the encoder
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Is Ready.");
        telemetry.update();
    }
    //</editor-fold>

}