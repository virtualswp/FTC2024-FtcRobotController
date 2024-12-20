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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Bob Teleop", group="Linear OpMode")
@Disabled
public class BobTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRearDriveMotor = null;
    private DcMotor rightRearDriveMotor = null;
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor collectorMotor = null;


    // Defines the speed to run the collector at.
    static final double collectorSpeed = 0.4;

    //Defines the speed to increase the collector at.
    static final double collectorSpeedInterval = 0.1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRearDriveMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightRearDriveMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "right_front");
        collectorMotor = hardwareMap.get(DcMotor.class, "collector");


        //gripper = hardwareMap.get(Servo.class, "gripperServo1");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRearDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        collectorMotor.setDirection(DcMotor.Direction.FORWARD);

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
                leftPower  = -gamepad1.left_stick_y ;
                rightPower = -gamepad1.right_stick_y ;


                leftRearDriveMotor.setPower(leftPower);
                leftFrontDriveMotor.setPower(leftPower);
                rightRearDriveMotor.setPower(rightPower);
                rightFrontDriveMotor.setPower(rightPower);
            }

            // Collector
            if (gamepad1.square == true){
                collectorMotor.setPower(collectorSpeed);
            }
            else if (gamepad1.circle == true){
                collectorMotor.setPower(-collectorSpeed);
            }
            else if (gamepad1.triangle == true){
                collectorMotor.setPower(0.0);
            }
            else if (gamepad1.cross == true){
                //Speed up / slow down
                double currentSpeed = collectorMotor.getPower();

                //Check if we are running forward or reverse
                if (currentSpeed > 0.0)
                {
                    //Forward
                    double newSpeed = currentSpeed + collectorSpeedInterval;
                    collectorMotor.setPower(newSpeed);
                }
                else {
                    //Backwards
                    double newSpeed = currentSpeed - collectorSpeedInterval;
                    collectorMotor.setPower(newSpeed);
                }
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

