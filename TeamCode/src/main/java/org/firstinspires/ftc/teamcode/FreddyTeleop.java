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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
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




@TeleOp(name="Freddy Teleop", group="Robot")
//@Disabled
public class FreddyTeleop extends LinearOpMode {

    /* Declare hardware variables */
    public DcMotor leftFrontDriveMotor = null; //the left drivetrain motor
    public DcMotor rightFrontDriveMotor = null; //the right drivetrain motor
    public DcMotor leftRearDriveMotor = null; //the left drivetrain motor
    public DcMotor rightRearDriveMotor = null; //the right drivetrain motor

    public DcMotor  armMotor    = null; //the arm motor
    public DcMotor  slideMotor = null;

    private CRServo collectorLeft = null;
    private CRServo collectorRight = null;

    private TouchSensor slideButton = null;      // The Viper Slide button at the top of the clip


    // Member variables
    private armPosition currentArmPosition = armPosition.retracted;         //The current arm position
    private armPosition targetArmPosition = armPosition.retracted;          //The target arm position
    private int basketArmMoveStep = 0;                                      //While moving the arm in, what stage it's in

    private driveMode currentDriveMode = driveMode.collection;              //The current drive mode
    private boolean isSlideButtonPressed = false;                           //Variable to determine if the slide button is being pressed


    //Enumerations
    private enum armPosition {
        retracted,
        collectUp,
        collectDown,
        lowBasket,
        highBasket,
        endgame
    }

    private enum driveMode {
        normal,
        collection
    }




    @Override
    public void runOpMode() {
        /* Define and Initialize Motors */
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDrive");          //Control Hub Motor Port 0
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDrive");        //Control Hub Motor Port 1
        leftRearDriveMotor = hardwareMap.get(DcMotor.class, "leftRearDrive");            //Control Hub Motor Port 2
        rightRearDriveMotor = hardwareMap.get(DcMotor.class, "rightRearDrive");          //Control Hub Motor Port 3

        collectorLeft = hardwareMap.get(CRServo.class, "collectorLeft");                 //Control Hub Servo Port 0
        collectorRight = hardwareMap.get(CRServo.class, "collectorRight");               //Control Hub Servo Port 1

        armMotor   = hardwareMap.get(DcMotor.class, "arm"); //the arm motor              //Expansion Hub Motor Port 0
        slideMotor = hardwareMap.get(DcMotor.class, "slide");                            //Expansion Hub Motor Port 1

        slideButton = hardwareMap.get(TouchSensor.class, "slideButton");              //Expansion Hub Sensor Port 0


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
        ((DcMotorEx) slideMotor).setCurrentAlert(5,CurrentUnit.AMPS);


        //Set the arm motor's motor encoder to 0
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the viper slide motor encode to 0
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            this.HandleTeleopDrive();
            this.HandleTeleopCollector();
            this.HandleTeleopArm();
            this.HandleSlideMotorSensors();

            this.HandleTeleopTelemetry();
        }
    }

    private void MoveArmToCollectPosition(){
        //Lift to collect up position

        //Determine the current position
        switch (this.currentArmPosition){
            case highBasket:
                //First move the slide in
                slideMotor.setTargetPosition(-2500);
                ((DcMotorEx) slideMotor).setVelocity(1700);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(slideMotor)){
                    //Next, move the arm down
                    armMotor.setTargetPosition(500);
                    ((DcMotorEx) armMotor).setVelocity(500);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Next, check if the arm is at the final position
                    if (this.isMotorAtPosition(armMotor)){
                        //Success
                        this.currentArmPosition = armPosition.collectUp;
                    }
                }
                break;
            case lowBasket:
                //First move the slide in
                slideMotor.setTargetPosition(-2500);
                ((DcMotorEx) slideMotor).setVelocity(500);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(slideMotor)){
                    //Next, move the arm down
                    armMotor.setTargetPosition(500);
                    ((DcMotorEx) armMotor).setVelocity(500);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Next, check if the arm is at the final position
                    if (this.isMotorAtPosition(armMotor)){
                        //Success
                        this.currentArmPosition = armPosition.collectUp;
                    }
                }
                break;
            default:
                //First move the arm up
                armMotor.setTargetPosition(500);
                ((DcMotorEx) armMotor).setVelocity(1900);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Next, check if the arm is up, if so, then slide out
                if (this.isMotorAtPosition(armMotor)){
                    slideMotor.setTargetPosition(-2500);
                    ((DcMotorEx) slideMotor).setVelocity(1900);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //Next, check if the slide is at the final position
                    if (this.isMotorAtPosition(slideMotor)){
                        //Success
                        this.currentArmPosition = armPosition.collectUp;
                    }
                }
                break;
        }
    }

    private void MoveArmToCollectDownPosition(){
        //Lift to collect down position

        // First, set the slide to the correct position
        slideMotor.setTargetPosition(-2500);
        ((DcMotorEx) slideMotor).setVelocity(1900);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (this.isMotorAtPosition(slideMotor)) {
            //Next, move the arm down but determine where we are coming from
            switch (this.currentArmPosition){
                case highBasket:
                    //Move down more slowly if coming from the high basket
                    armMotor.setTargetPosition(200);
                    ((DcMotorEx) armMotor).setVelocity(500);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;
                case lowBasket:
                    //Move down more slowly if coming from the low basket
                    armMotor.setTargetPosition(0);
                    ((DcMotorEx) armMotor).setVelocity(800);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;
                default:
                    //Move down quickly if moving from collect up, or retract
                    armMotor.setTargetPosition(200);
                    ((DcMotorEx) armMotor).setVelocity(700);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
            }

            if (this.isMotorAtPosition(armMotor)){
                //Success
                this.currentArmPosition = armPosition.collectDown;
            }
        }
    }

    private void MoveArmToRetractedPosition(){
        //First move the slide back until it touches the button
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(0.5);

        //Check if the button is being pressed
        if (this.isSlideButtonPressed){
            //Reset the slide position
            slideMotor.setPower(0);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(0);

            //Next, move the arm down
            armMotor.setTargetPosition(0);
            ((DcMotorEx) armMotor).setVelocity(300);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (this.isMotorAtPosition(armMotor)){
                //Success
                this.currentArmPosition = armPosition.retracted;
            }
        }
    }

    private void HandleTeleopArm(){
        //--------------Arm / Slide-----------------------------
        if (gamepad2.x){
            //Move to high basket position
            this.basketArmMoveStep = 0;
            this.targetArmPosition = armPosition.highBasket;
        }
        else if (gamepad2.b){
            //Move to the low basket position
            this.basketArmMoveStep = 0;
            this.targetArmPosition = armPosition.lowBasket;
        }
        else if (gamepad2.y) {
            //Move to collect up position
            this.targetArmPosition = armPosition.collectUp;
        }
        else if (gamepad2.a) {
            //Move to the collect down position
            if (this.currentArmPosition != armPosition.retracted)
                this.targetArmPosition = armPosition.collectDown;
        }
        else if (gamepad2.right_bumper){
            //Retract the arm to starting position
            this.targetArmPosition = armPosition.retracted;
        }
        else if (gamepad2.left_bumper){
            //Move to the end game position
            this.basketArmMoveStep = 0;
            this.targetArmPosition = armPosition.endgame;
        }

        //Next, check if we need to move the arm
        if (!this.isArmAtTargetPosition()){
            switch (this.targetArmPosition){
                case collectUp:
                    this.MoveArmToCollectPosition();
                    break;
                case collectDown:
                    this.MoveArmToCollectDownPosition();
                    break;
                case highBasket:
                    this.MoveArmToHighBasketPosition();
                    break;
                case lowBasket:
                    this.MoveArmToLowBasketPosition();
                    break;
                case retracted:
                    this.MoveArmToRetractedPosition();
                    break;
                case endgame:
                    this.MoveArmToEndGamePosition();
                    break;
            }
        }
    }



    private void MoveArmToHighBasketPosition(){
        switch (basketArmMoveStep){
            case 0:     //First move the arm up 1/2 way
                armMotor.setTargetPosition(750);
                ((DcMotorEx) armMotor).setVelocity(700);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(armMotor)){
                    this.basketArmMoveStep = 1;
                }

                break;
            case 1:     //Next move the slide out all the way
                slideMotor.setTargetPosition(-5700);
                ((DcMotorEx) slideMotor).setVelocity(1200);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(slideMotor)){
                    this.basketArmMoveStep = 2;
                }

                break;
            case 2:     // Next, move the arm up fully
                armMotor.setTargetPosition(1700);
                ((DcMotorEx) armMotor).setVelocity(700);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(armMotor)){
                    this.basketArmMoveStep = 3;
                    this.currentArmPosition = armPosition.highBasket;
                }

                break;
        }
    }

    private void MoveArmToLowBasketPosition(){
        switch (basketArmMoveStep){
            case 0:     //First move the arm up 1/2 way
                armMotor.setTargetPosition(1300);
                ((DcMotorEx) armMotor).setVelocity(700);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(armMotor)){
                    this.basketArmMoveStep = 1;
                }

                break;
            case 1:     //Next move the slide out 1/2 way
                slideMotor.setTargetPosition(-2500);
                ((DcMotorEx) slideMotor).setVelocity(700);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(slideMotor)){
                    this.basketArmMoveStep = 2;
                    this.currentArmPosition = armPosition.lowBasket;
                }
        }
    }

    private void MoveArmToEndGamePosition(){
        switch (basketArmMoveStep){
            case 0:     //First move the arm up 1/2 way
                armMotor.setTargetPosition(900);
                ((DcMotorEx) armMotor).setVelocity(700);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(armMotor)){
                    this.basketArmMoveStep = 1;
                }

                break;
            case 1:     //Next move the slide out 1/4 way
                slideMotor.setTargetPosition(-800);
                ((DcMotorEx) slideMotor).setVelocity(700);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (this.isMotorAtPosition(slideMotor)){
                    this.basketArmMoveStep = 2;
                    this.currentArmPosition = armPosition.lowBasket;
                }
        }
    }

    private void HandleSlideMotorSensors(){
        this.isSlideButtonPressed = slideButton.isPressed();
    }

    private void HandleTeleopDrive(){
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0.0;
        double rightPower = 0.0;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double strafeRight = gamepad1.right_trigger;
        double strafeLeft = gamepad1.left_trigger;

        boolean rightUpStrafe = gamepad1.dpad_up;
        boolean rightDownStrafe = gamepad1.dpad_right;
        boolean leftUpStrafe = gamepad1.dpad_left;
        boolean leftDownStrafe = gamepad1.dpad_down;

        boolean collectorModeDisable = gamepad1.y;
        boolean collectorModeEnable = gamepad1.a;

        if (collectorModeDisable == true){
            this.currentDriveMode = driveMode.normal;
        }
        else if (collectorModeEnable == true){
            this.currentDriveMode = driveMode.collection;
        }


        if (strafeLeft > 0) {
            ChassisMotorValues c = new ChassisMotorValues();
            c = this.strafeLeft(strafeLeft);

            leftRearDriveMotor.setPower(c.leftRear);
            leftFrontDriveMotor.setPower(c.leftFront);
            rightRearDriveMotor.setPower(c.rightRear);
            rightFrontDriveMotor.setPower(c.rightFront);
        }
        else if (strafeRight > 0) {
            ChassisMotorValues c = new ChassisMotorValues();
            c = this.strafeRight(strafeRight);
            leftRearDriveMotor.setPower(c.leftRear);
            leftFrontDriveMotor.setPower(c.leftFront);
            rightRearDriveMotor.setPower(c.rightRear);
            rightFrontDriveMotor.setPower(c.rightFront);
        }
        else if (gamepad1.left_bumper){
            diagonalStrafe(rightUpStrafe, rightDownStrafe, leftUpStrafe, leftDownStrafe);
        }
        else
        {
            // Tank Mode uses one stick to control each wheel.
            if (this.currentDriveMode == driveMode.collection){
                // Collect drive mode (slow down)
                float collectorModeFactor = 0.7F;
                leftPower  = gamepad1.left_stick_y * collectorModeFactor;
                rightPower = -gamepad1.right_stick_y * collectorModeFactor;
            }
            else {
                // Normal Drive mode (full speed)
                leftPower  = gamepad1.left_stick_y ;
                rightPower = -gamepad1.right_stick_y ;
            }



            leftRearDriveMotor.setPower(leftPower);
            leftFrontDriveMotor.setPower(leftPower);
            rightRearDriveMotor.setPower(rightPower);
            rightFrontDriveMotor.setPower(rightPower);
        }
    }

    private void HandleTeleopCollector(){
        float collectorInput = gamepad2.left_trigger;
        float collectorOutput = gamepad2.right_trigger;

        if (collectorInput > 0){
            collectorLeft.setPower((double)collectorInput);
            collectorRight.setPower(-(double)collectorInput);
        }
        else if (collectorOutput > 0){
            collectorLeft.setPower(-(double)collectorOutput);
            collectorRight.setPower((double)collectorOutput);
        }
        else {
            collectorLeft.setPower(0.0);
            collectorRight.setPower(0.0);
        }
    }

    public boolean isMotorAtPosition(DcMotor motor)
    {
        if (!motor.isBusy()){
            telemetry.addData("motor: ", motor.isBusy());
            return true;
        }

        int varianceFactor = 10;

        int currentMotorPosition = motor.getCurrentPosition();
        int targetMotorPosition = motor.getTargetPosition();

        telemetry.addData("currentMotorPosition: ", currentMotorPosition);
        telemetry.addData("targetMotorPosition: ", targetMotorPosition);

        int error = Math.abs(targetMotorPosition - currentMotorPosition);

        telemetry.addData("error: ", error);
        telemetry.addData("isMotorAtPosition", (error <= varianceFactor));

        return error <= varianceFactor;
    }

    private boolean isArmAtTargetPosition(){
        return (this.currentArmPosition == this.targetArmPosition);
    }

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
        if (rightUpStrafe == true)
        {
            leftFrontDriveMotor.setPower(0.45);
            rightRearDriveMotor.setPower(-0.45);
        }
        else if (rightDownStrafe == true){
            rightFrontDriveMotor.setPower(-0.45);
            leftRearDriveMotor.setPower(0.45);
        }
        else if (leftUpStrafe == true){
            rightFrontDriveMotor.setPower(0.45);
            leftRearDriveMotor.setPower(-0.45);
        }
        else if (leftDownStrafe == true){
            leftFrontDriveMotor.setPower(-0.45);
            rightRearDriveMotor.setPower(0.45);
        }
        else {
            leftFrontDriveMotor.setPower(0);
            rightRearDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftRearDriveMotor.setPower(0);
        }
    }


    private void HandleTeleopTelemetry(){
        /* Check to see if our arm and slider is over the current limit, and report via telemetry. */
        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("ARM MOTOR EXCEEDED CURRENT LIMIT!");
        }

        if (((DcMotorEx) slideMotor).isOverCurrent()){
            telemetry.addLine("SLIDE MOTOR EXCEEDED CURRENT LIMIT!");
        }


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("Arm Target: ", armMotor.getTargetPosition());
        telemetry.addData("Arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("Slide Target: ", slideMotor.getTargetPosition());
        telemetry.addData("Slide Encoder: ", slideMotor.getCurrentPosition());
        telemetry.addData("Arm Position", this.currentArmPosition);
        telemetry.update();
    }

}