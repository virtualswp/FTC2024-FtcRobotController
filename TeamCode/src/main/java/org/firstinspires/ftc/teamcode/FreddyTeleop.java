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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    /* Declare OpMode members. */
    public DcMotor leftFrontDriveMotor = null; //the left drivetrain motor
    public DcMotor rightFrontDriveMotor = null; //the right drivetrain motor
    public DcMotor leftRearDriveMotor = null; //the left drivetrain motor
    public DcMotor rightRearDriveMotor = null; //the right drivetrain motor

    public DcMotor  armMotor    = null; //the arm motor
    public DcMotor  slideMotor = null;

    private CRServo collectorLeft = null;
    private CRServo collectorRight = null;

    // Member variables
    private armPosition currentArmPosition = armPosition.retracted;         //The current arm position


    //Enumerations
    private enum armPosition {
        retracted,
        collectUp,
        collectDown,
        highBasket
    }




    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    //The number of ticks for the arm motor (From the gobilda website)
    final double armMotorTicks = 5281.1;



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

            //------------Drive---------------------
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
                leftPower  = gamepad1.left_stick_y ;
                rightPower = -gamepad1.right_stick_y ;


                leftRearDriveMotor.setPower(leftPower);
                leftFrontDriveMotor.setPower(leftPower);
                rightRearDriveMotor.setPower(rightPower);
                rightFrontDriveMotor.setPower(rightPower);
            }

            //-----------End Drive-------------------



            //--------------Arm / Slide-----------------------------
            if (gamepad2.x){
                //Lift to top basket

                armMotor.setTargetPosition(1700);
                ((DcMotorEx) armMotor).setVelocity(700);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slideMotor.setTargetPosition(-5700);
                ((DcMotorEx) slideMotor).setVelocity(2000);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.currentArmPosition = armPosition.highBasket;
            }
            else if (gamepad2.y) {
                //Lift to collect up position
                armMotor.setTargetPosition(550);
                ((DcMotorEx) armMotor).setVelocity(500);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slideMotor.setTargetPosition(-2500);
                ((DcMotorEx) slideMotor).setVelocity(1200);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.currentArmPosition = armPosition.collectUp;
            }
            else if (gamepad2.a) {
                //Lift to collect down position
                armMotor.setTargetPosition(0);
                ((DcMotorEx) armMotor).setVelocity(500);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slideMotor.setTargetPosition(-2500);
                ((DcMotorEx) slideMotor).setVelocity(1900);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.currentArmPosition = armPosition.collectDown;
            }
            else if (gamepad2.right_bumper){
                //Retract all the way
                armMotor.setTargetPosition(0);
                ((DcMotorEx) armMotor).setVelocity(300);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slideMotor.setTargetPosition(0);
                ((DcMotorEx) slideMotor).setVelocity(1200);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.currentArmPosition = armPosition.retracted;
            }

            //Resets the viper slide back to 0 if the Viper Slide is high current
            if (gamepad2.b){
                slideMotor.setTargetPosition(0);
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                this.currentArmPosition = armPosition.retracted;
            }
            //---------------End Arm------------------------




            //---------------Viper Slide------------------
            double slidePower = gamepad2.left_stick_y;

            if (slidePower > 0) {
                //slideMotor.setPower(0.5);
            }
            else if (slidePower < 0) {
                //slideMotor.setPower(-0.5);
            }
            else {
                //slideMotor.setPower(0.0);
            }
            //-----------End Viper Slide-----------------



            // ------------- Collector -----------------------
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
            //--------------End Collector----------------------






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
            rightRearDriveMotor.setPower(0.45);
        }
        else if (rightDownStrafe == true){
            rightFrontDriveMotor.setPower(-0.45);
            leftRearDriveMotor.setPower(-0.45);
        }
        else if (leftUpStrafe == true){
            rightFrontDriveMotor.setPower(0.45);
            leftRearDriveMotor.setPower(0.45);
        }
        else if (leftDownStrafe == true){
            leftFrontDriveMotor.setPower(-0.45);
            rightRearDriveMotor.setPower(-0.45);
        }
        else {
            leftFrontDriveMotor.setPower(0);
            rightRearDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftRearDriveMotor.setPower(0);
        }
    }


}