package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Brad Teleop", group="Linear OpMode")
public class BradTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private CRServo liftArm = null;
    private CRServo collectorLeft = null;
    private CRServo collectorRight = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        liftArm = hardwareMap.get(CRServo.class, "liftarm");
        //collectorLeft = hardwareMap.get(CRServo.class, "collectorleft");
        //collectorRight = hardwareMap.get(CRServo.class, "collectorright");


        //gripper = hardwareMap.get(Servo.class, "gripperServo1");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower = 0.0;
            double rightPower = 0.0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double strafeRight = gamepad1.right_trigger;
            double strafeLeft = gamepad1.left_trigger;

            boolean rightUpStrafe = gamepad1.dpad_up;
            boolean rightDownStrafe = gamepad1.dpad_right;
            boolean leftUpStrafe = gamepad1.dpad_left;
            boolean leftDownStrafe = gamepad1.dpad_down;

            boolean grab = gamepad1.x;
            boolean unGrab = gamepad1.b;


         /*   if (grab == true) {
                gripper.setPosition(0.1);
            } else if (unGrab == true) {
                gripper.setPosition(0.75);
            }
            else {
                gripper.setPosition(0.5);
            }*/


            boolean liftUp = gamepad1.dpad_up;
            boolean liftDown = gamepad1.dpad_down;

            if (liftUp == true) {
                liftArm.setPower(0.5);
            }
            else
            {
                liftArm.setPower(0);
            }
            if (liftDown == true) {
                liftArm.setPower(-0.25);
            }
            else
            {
                liftArm.setPower(0);
            }

          /*  boolean collectorInput = gamepad1.x;
            boolean collectorOutput = gamepad1.b;
            boolean collectorStop = gamepad1.y;

            if (collectorInput == true){
                collectorLeft.setPower(1.0);
                collectorRight.setPower(-1.0);
            }
            else if (collectorOutput == true){
                collectorLeft.setPower(-1.0);
                collectorRight.setPower(1.0);
            }
            else if (collectorStop == true){
                collectorLeft.setPower(0.0);
                collectorRight.setPower(0.0);
            }*/




            if (strafeLeft > 0) {
                ChassisMotorValues c = new ChassisMotorValues();
                c = this.strafeLeft(strafeLeft);

                leftDrive.setPower(c.leftRear);
                leftFront.setPower(c.leftFront);
                rightDrive.setPower(c.rightRear);
                rightFront.setPower(c.rightFront);
            }
            else if (strafeRight > 0) {
                ChassisMotorValues c = new ChassisMotorValues();
                c = this.strafeRight(strafeRight);

                leftDrive.setPower(c.leftRear);

                leftFront.setPower(c.leftFront);

                rightDrive.setPower(c.rightRear);

                rightFront.setPower(c.rightFront);
            }
            else if (gamepad1.left_bumper){
                diagonalStrafe(rightUpStrafe, rightDownStrafe, leftUpStrafe, leftDownStrafe);
            }
            else
            {
                // Tank Mode uses one stick to control each wheel.

                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


                leftPower  = -gamepad1.left_stick_y ;
                rightPower = -gamepad1.right_stick_y ;


                leftDrive.setPower(leftPower);
                leftFront.setPower(leftPower);
                rightDrive.setPower(rightPower);
                rightFront.setPower(rightPower);
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    public ChassisMotorValues strafeRight(double strafePower) {
        ChassisMotorValues result = new ChassisMotorValues();

        result.leftRear = -strafePower;
        result.leftFront = strafePower;
        result.rightRear = strafePower;
        result.rightFront = -strafePower;

        return result;
    }
    public ChassisMotorValues strafeLeft(double strafePower) {
        ChassisMotorValues result = new ChassisMotorValues();

        result.leftRear = strafePower;
        result.leftFront = -strafePower;
        result.rightRear = -strafePower;
        result.rightFront = strafePower;

        return result;
    }

    public void diagonalStrafe(boolean rightUpStrafe, boolean rightDownStrafe, boolean leftUpStrafe, boolean leftDownStrafe) {
        if (rightUpStrafe == true)
        {
            leftFront.setPower(0.45);
            rightDrive.setPower(0.45);
        }
        else if (rightDownStrafe == true){
            rightFront.setPower(-0.45);
            leftDrive.setPower(-0.45);
        }
        else if (leftUpStrafe == true){
            rightFront.setPower(0.45);
            leftDrive.setPower(0.45);
        }
        else if (leftDownStrafe == true){
            leftFront.setPower(-0.45);
            rightDrive.setPower(-0.45);
        }
        else {
            leftFront.setPower(0);
            rightDrive.setPower(0);
            rightFront.setPower(0);
            leftDrive.setPower(0);
        }
    }
}
