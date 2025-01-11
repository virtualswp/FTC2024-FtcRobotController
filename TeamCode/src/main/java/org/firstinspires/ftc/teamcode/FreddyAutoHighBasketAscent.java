package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Freddy Auto - High Basket Ascent", group="Linear OpMode")
public class FreddyAutoHighBasketAscent extends LinearOpMode {

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

    private IMU imu;

    //</editor-fold>

    //<editor-fold desc="Member Variables">

    private FreddyAutoHighBasketAscent.armPosition currentArmPosition = FreddyAutoHighBasketAscent.armPosition.home;         //The current arm position

    private FreddyAutoHighBasketAscent.armPosition targetArmPosition = FreddyAutoHighBasketAscent.armPosition.home;          //The target arm position

    private FreddyAutoHighBasketAscent.slidePosition currentSlidePosition = FreddyAutoHighBasketAscent.slidePosition.home;            // The current slide position

    private FreddyAutoHighBasketAscent.slidePosition targetSlidePosition = FreddyAutoHighBasketAscent.slidePosition.home;             // The target slide position

    private FreddyAutoHighBasketAscent.driveMode currentDriveMode = FreddyAutoHighBasketAscent.driveMode.normal;              //The current drive mode

    private boolean isArmFrontButtonPressed = false;                     //Variable to determine if the front arm button is being pressed

    private boolean isArmBackButtonPressed = false;                     //Variable to determine if the back arm button is being pressed

    private boolean isArmSlideSwitchPressed = false;                    //Variable to determine if the slide's magnetic switch sensor is pressed

    private boolean isArmSlideBackButtonPressed = false;                //Variable to determine if the slide arm's back button / touch sensor is pressed

    private ElapsedTime runtime = new ElapsedTime();

    //</editor-fold>

    //<editor-fold desc="Constants">

    private static final double ENCODER_ZERO_POWER = 0.1;               //The amount of power to have the motor use to brake (hold) the motor position.
    private static final int SLIDE_LOW_BASKET = 1500;                   //The degrees the encoder needs to move the slide motor to get to the low basket
    private static final int SLIDE_HIGH_BASKET = 3700;                  //The degrees the encoder needs to move the slide motor to get to the high basket

    private static final int SLIDE_HIGH_BASKET_MIN = 3550;              //The minimum height the magnetic switch sensor can be valid at

    //Napoleon = 450, Freddy = 550
    private static final int SLIDE_COLLECT_OUT = 550;                   //The degrees the encoder needs to move the slide motor to get to the collect out position

    private static final int SLIDE_COLLECT_OUT_SPEED = 700;             //The velocity to move the slide out to collect out.

    private static final int SLIDE_HOME_RESET = -200;                   // The position to move the slide motor past the home (0) position to account for any variance with the encoder

    private static final int ARM_COLLECT_DELIVERY = -3700;             //The degrees to move the arm straight up to the delivery up position

    private static final int ARM_HOME_RESET = 400;                     // The position to move the arm motor past the home (0) position to account for any variance with the encoder

    private static final double HAND_OPEN_POSITION = 0.0;               // The servo position for the hand to be fully open.

    //Napoleon = 0.57, Freddy = 0.64
    private static final double HAND_CLOSED_POSITION = 0.64;            // The servo position for the hand to be fully closed.

    // Napoleon = 0.70, Freddy = 0.67
    private static final double WRIST_DOWN_POSITION = 0.70;             // The servo position for the wrist to be fully down.

    private static final double WRIST_BACK_POSITION = 0.0;              // The servo position for the wrist to be fully back.

    private static final double WRIST_DELIVERY_POSITION = 0.30;         // The servo position for the wrist when delivering to the baskets

    private static final double WRIST_COLLECT_UP_POSITION = 0.5;        // The servo position for the wrist when moving to collect up position (slightly back to go over the bar)



    //</editor-fold>

    //<editor-fold desc="Gyro Steering Constants">

    // Constants for gyro steering drive calculations
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

    //</editor-fold>

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

    //<editor-fold desc="Op Mode & Handlers">

    @Override
    public void runOpMode() {
        // Initialize hardware
        this.ConfigureHardware();

        waitForStart();

        //Move to grippers default positions
        this.SetHardwareDefaultPositions();

        // Drive forward to move off the wall
        gyroDrive(DRIVE_SPEED, 3.0, 0.0);

        // Strafe to the left and park in the net zone
        gyroStrafe(STRAFE_SPEED, 26.0, 0.0, strafeDirection.left);

        // Turn right from original direction to line up the basket
        gyroTurn(TURN_SPEED, -45.0);

        // Strafe to the left to center with the basket
        gyroStrafe(STRAFE_SPEED, 7.0, -45.0, strafeDirection.left);

        //Move the arm to the delivery position
        this.MoveArmToDeliveryUpPosition();

        //Move the arm up to the basket for delivery
        this.MoveSlideToHighBasketPosition();

        //Back Up to the basket slowly
        gyroDrive((DRIVE_SPEED - 0.2), -3, -45.0);

        this.HandOpen();

        sleep(1000);

        this.HandClose();

        //Move Forward To Close Up The Arm
        gyroDrive((DRIVE_SPEED - 0.2), 5.25, -45.0);

        //Put da SLide D00wn
        this.MoveSlideToHomePosition();

        //sleep(1000);

        //Move forward a little bit
        gyroDrive(DRIVE_SPEED, 15.0, -45.0);

        //set it up to drive towards ascent zone
        gyroTurn(DRIVE_SPEED, 0.0);

        //drive to acsent zone
        gyroDrive(DRIVE_SPEED, 30.0, 0.0);

        //turn around to get ready to touch it
        gyroTurn(DRIVE_SPEED, 90);

        //back up and touch it
        gyroDrive(DRIVE_SPEED, -11, 90);

        //Move the arm down to home position
        this.MoveArmToHomePosition();

        //Move the arm up for ascent
        this.RunAscentArmForTime(1.1);

        //final touch
        gyroDrive((DRIVE_SPEED - 0.2), -5, 90);

        //Auto reset all hardware for teleop
        this.ResetForTeleop();

        //Open the gripper for teleop
        this.HandOpen();



        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Success.");
        telemetry.update();

        sleep(5000);
    }


    private void HandleArmSensors() {
        this.isArmFrontButtonPressed = armButtonFront.isPressed();
        this.isArmBackButtonPressed = armButtonRear.isPressed();
        this.isArmSlideSwitchPressed = armSlideSwitch.isPressed();
        this.isArmSlideBackButtonPressed = armSlideButtonRear.isPressed();
    }

    //</editor-fold>


    //<editor-fold desc="Arm Methods ">

    private void MoveArmToHomePosition() {
        while (opModeIsActive() && this.currentArmPosition != armPosition.home) {
            this.HandleArmSensors();

            //First, determine the current position
            switch (this.currentArmPosition) {
                case collectUp:
                    // Move the arm down to the down up position using the encoder
                    this.slideArmMotor.setTargetPosition(ARM_HOME_RESET);
                    ((DcMotorEx) this.slideArmMotor).setVelocity(300);
                    this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Next, check if the arm is at the final position (Either from the encoder or the button press)
                    if (this.isMotorAtPosition(slideArmMotor) || this.isArmFrontButtonPressed) {
                        //Set the encoder position to the current position to hold the arm up
                        this.slideArmMotor.setTargetPosition(this.slideArmMotor.getCurrentPosition());
                        this.slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                        //Success
                        this.currentArmPosition = FreddyAutoHighBasketAscent.armPosition.home;
                    }

                    break;
                case collectOut:
                    // Move the slide in the home position using the encoder until the button is pressed
                    this.slideMotor.setTargetPosition(SLIDE_HOME_RESET);
                    ((DcMotorEx) this.slideMotor).setVelocity(300);
                    this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Next, check if the arm is at the final position
                    if (this.isMotorAtPosition(slideMotor) || this.isArmSlideBackButtonPressed) {
                        //If the back button is being pressed, reset the encoder to 0
                        if (this.isArmSlideBackButtonPressed) {
                            //Set the encoder position to the current position to hold the arm up
                            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            slideMotor.setTargetPosition(0);
                            slideMotor.setPower(ENCODER_ZERO_POWER);
                            this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        } else {
                            //Set the encoder position to the current position to hold the arm up
                            this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                            this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }

                        //Set the encoder position to the current position to hold the arm up
                        this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                        this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        //Success
                        this.currentArmPosition = FreddyAutoHighBasketAscent.armPosition.home;
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
                        this.currentArmPosition = FreddyAutoHighBasketAscent.armPosition.home;
                    }

                    break;

                case home:
                    //Already here, shouldn't happen
                    break;
            }
        }
    }

    private void MoveArmToDeliveryUpPosition() {

        while (opModeIsActive() && this.currentArmPosition != armPosition.deliveryUp) {
            this.HandleArmSensors();

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
                        this.currentArmPosition = FreddyAutoHighBasketAscent.armPosition.deliveryUp;
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
                            this.currentArmPosition = FreddyAutoHighBasketAscent.armPosition.deliveryUp;
                        }
                    }
                }
                break;

                case deliveryUp:
                    //This shouldn't happen, it's already in delivery up position
                    break;
            }
        }
    }

    //</editor-fold>

    //<editor-fold desc="Basket Methods ">

    private void MoveSlideToHighBasketPosition() {

        while (opModeIsActive() && currentSlidePosition != FreddyAutoHighBasketAscent.slidePosition.highBasket) {
            this.HandleArmSensors();

            slideMotor.setTargetPosition(SLIDE_HIGH_BASKET);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Next, determine where the side is at so we can throttle the speed of the slide movement
            int degreesUntilTarget = Math.abs(this.slideMotor.getTargetPosition() - this.slideMotor.getCurrentPosition());

            if (degreesUntilTarget <= 500) {
                // Run at a slow speed
                ((DcMotorEx) this.slideMotor).setVelocity(350);
            } else if (degreesUntilTarget <= 1000) {
                // Run at a medium speed
                ((DcMotorEx) this.slideMotor).setVelocity(650);
            } else {
                //Run at full speed
                ((DcMotorEx) this.slideMotor).setVelocity(2000);
            }

            if (this.isMotorAtPosition(slideMotor) ||
                    (this.isArmSlideSwitchPressed && (slideMotor.getCurrentPosition() >= SLIDE_HIGH_BASKET_MIN))) {
                //Set the encoder position to the current position to hold the slide up
                this.slideMotor.setTargetPosition(this.slideMotor.getCurrentPosition());
                this.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.currentSlidePosition = FreddyAutoHighBasketAscent.slidePosition.highBasket;
            }
        }
    }

    private void MoveSlideToHomePosition() {
        while (opModeIsActive() && currentSlidePosition != FreddyAutoHighBasketAscent.slidePosition.home) {
            this.HandleArmSensors();

            //Move the wrist to the delivery position
            this.gripperWrist.setPosition(WRIST_DOWN_POSITION);

            slideMotor.setTargetPosition(0);
            ((DcMotorEx) slideMotor).setVelocity(1500);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (this.isMotorAtPosition(slideMotor)) {
                this.currentSlidePosition = FreddyAutoHighBasketAscent.slidePosition.home;
            }
        }
    }

    //</editor-fold>




    private void RunAscentArmForTime(double seconds) {
        runtime.reset();
        final double rightArmPower = 0.7;

        while (opModeIsActive() && runtime.seconds() < seconds) {
            this.armMotorRight.setPower(-rightArmPower);
        }

        this.armMotorRight.setPower(0.0);

    }

    //<editor-fold desc="Gripper Methods ">

    private void HandOpen(){
        //Set the hand gripper to an open position to start
        this.gripperHand.setPosition(HAND_OPEN_POSITION);                //0.0 = All the way open, 1.0 is all the way closed.
    }

    private void HandClose(){
        this.gripperHand.setPosition(HAND_CLOSED_POSITION);
    }

    private void WristBack(){
        this.gripperWrist.setPosition(WRIST_BACK_POSITION);                //0.0 = All the way down, 1.0 is all the way back.
    }

    private void WristForward(){
        this.gripperWrist.setPosition(WRIST_DOWN_POSITION);                //0.0 = All the way down, 1.0 is all the way back.
    }

    //</editor-fold>



    //<editor-fold desc="Utility Methods">

    private void CheckInitialArmState() {
        /* This method will check where the arm is starting at (from the prior auto code or teleop) */
        if (this.isArmFrontButtonPressed) {
            this.currentArmPosition = FreddyAutoHighBasketAscent.armPosition.home;
        }
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

    private void SetHardwareDefaultPositions() {
        /* This method will set any hardware default positions */

        //Set the hand gripper to an open position to start
        this.gripperHand.setPosition(HAND_CLOSED_POSITION);                //0.0 = All the way open, 1.0 is all the way closed.
        this.gripperWrist.setPosition(WRIST_BACK_POSITION);                //0.0 = All the way down, 1.0 is all the way back.
    }

    private void ResetForTeleop(){
        //Turn off the encoders for the arm motors
        slideArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Create a variable to check if all sensors are reset
        boolean hardwareReset = false;

        while (hardwareReset == false){
            this.HandleArmSensors();

            //Check if the arm is fully down
            if (!this.isArmFrontButtonPressed){
                //Move the motor down
                this.slideArmMotor.setPower(0.1);
            }
            else{
                //Stop the motor
                this.slideArmMotor.setPower(0.0);
            }

            //Check if the slide is fully back
            if (!this.isArmSlideBackButtonPressed){
                //Move the motor backwards
                this.slideMotor.setPower(-0.2);
            }
            else{
                //Stop the motor
                this.slideMotor.setPower(0.0);
            }

            //Add telemetry to just see sensors
            telemetry.addData("isArmSlideBackButtonPressed", isArmSlideBackButtonPressed);
            telemetry.addData("isArmFrontButtonPressed", isArmFrontButtonPressed);
            telemetry.update();

            //Check if everything is reset
            if (this.isArmFrontButtonPressed && this.isArmSlideBackButtonPressed){
                hardwareReset = true;
            }
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
        leftFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
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

        // Only reset encoders on rear motors
        leftRearDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set run modes
        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set the arm motor to use the encoder
        slideArmMotor.setTargetPosition(0);
        slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the slide motor to use the encoder
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                IMU_ORIENTATION)));


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Is Ready.");
        telemetry.update();
    }

    //</editor-fold>

    //<editor-fold desc="Gyro Steering Example Code">

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
            telemetry.addData("Left Rear Pos", leftRearDriveMotor.getCurrentPosition());
            telemetry.addData("Right Rear Pos", rightRearDriveMotor.getCurrentPosition());
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
            telemetry.addData("Left Rear Pos", leftRearDriveMotor.getCurrentPosition());
            telemetry.addData("Right Rear Pos", rightRearDriveMotor.getCurrentPosition());
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
        if (strafeDirection == FreddyAutoHighBasketAscent.strafeDirection.left){
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
        leftFrontDriveMotor.setPower(powers[0]);
        rightFrontDriveMotor.setPower(powers[1]);
        leftRearDriveMotor.setPower(powers[2]);
        rightRearDriveMotor.setPower(powers[3]);
    }

    private void resetRearEncoders() {
        leftRearDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean isDriveComplete(int targetTicks) {
        // Average only the rear encoders
        int avgPosition = (Math.abs(leftRearDriveMotor.getCurrentPosition()) +
                Math.abs(rightRearDriveMotor.getCurrentPosition())) / 2;
        return Math.abs(avgPosition) >= Math.abs(targetTicks);
    }

    private boolean isStrafeComplete(int targetTicks) {
        // For strafing, check both rear encoders
        // Note: When strafing, rear wheels will rotate in opposite directions
        int leftPos = Math.abs(leftRearDriveMotor.getCurrentPosition());
        int rightPos = Math.abs(rightRearDriveMotor.getCurrentPosition());
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

    //</editor-fold>
}