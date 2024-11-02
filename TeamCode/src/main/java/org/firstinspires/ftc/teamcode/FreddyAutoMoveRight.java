/* Copyright (c) 2017 FIRST. All rights reserved.
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

//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

//Imports from auto example code
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//Imports from FreddyTeleop
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;



/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */






@Autonomous(name="Freddy Auto - Move Right", group="Robot")
//@Disabled
public class FreddyAutoMoveRight extends LinearOpMode {

    /* Declare OpMode members. */
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

    //private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer


    // Member variables
    private double oldTime = 0;                                             //Used for odometry
    private boolean isSlideButtonPressed = false;                           //Variable to determine if the slide button is being pressed

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     test_move_speed = 0.5;
    static final double     test_spin_speed    = 0.2;





//    Used in Auto examples, may be able to remove them
//    static final double     FORWARD_SPEED = 0.6;
//    static final double     TURN_SPEED    = 0.5;

    //Enumerations
    private enum armPosition {
        retracted,
        collectUp,
        collectDown,
        highBasket
    }

    private enum driveMode {
        normal,
        collection
    }


    @Override
    public void runOpMode() {
        this.ConfigureHardware();

        // Wait for the game to start (driver presses START)
        waitForStart();

        //Pose2D startingPosition = odo.getPosition();

        this.RunCollectorIntakeForTime(1.0);
        this.DriveStrafeRight(2.5);
        this.DriveStop();

        //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", startingPosition.getX(DistanceUnit.MM), startingPosition.getY(DistanceUnit.MM), startingPosition.getHeading(AngleUnit.DEGREES));
        //telemetry.addData("Starting Position", data);

        //Pose2D endingPosition = odo.getPosition();
        //data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", endingPosition.getX(DistanceUnit.MM), endingPosition.getY(DistanceUnit.MM), endingPosition.getHeading(AngleUnit.DEGREES));
        //telemetry.addData("Ending Position", data);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(5000);
    }



    private void RunCollectorIntakeForTime(double Seconds){
        double collectorOutput = 1.0;

        collectorLeft.setPower(-collectorOutput);
        collectorRight.setPower(collectorOutput);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < Seconds)) {
            //Placeholder...
        }

        collectorLeft.setPower(0.0);
        collectorRight.setPower(0.0);
    }


    public void DriveStrafeRight(double Seconds) {
        ChassisMotorValues result = new ChassisMotorValues();

        double strafePower = 0.3;
        result.leftFront = -strafePower;    //Should move backwards
        result.rightFront = -strafePower;    //Should move forwards
        result.leftRear = strafePower;     //Should move forwards
        result.rightRear = strafePower;     //Should move backwards

        runtime.reset();
        //odo.resetPosAndIMU();
        //Pose2D endingPosition = odo.getPosition();
        while (opModeIsActive() && ((runtime.seconds() < Seconds))) {
            leftRearDriveMotor.setPower(result.leftRear);
            leftFrontDriveMotor.setPower(result.leftFront);
            rightRearDriveMotor.setPower(result.rightRear);
            rightFrontDriveMotor.setPower(result.rightFront);
        }
    }



    private void DriveStop(){
        leftRearDriveMotor.setPower(0);
        leftFrontDriveMotor.setPower(0);
        rightRearDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
    }






    private void ConfigureHardware(){
        /* Define and Initialize Motors */
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDrive");          //Control Hub Motor Port 0
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDrive");        //Control Hub Motor Port 1
        leftRearDriveMotor = hardwareMap.get(DcMotor.class, "leftRearDrive");            //Control Hub Motor Port 2
        rightRearDriveMotor = hardwareMap.get(DcMotor.class, "rightRearDrive");          //Control Hub Motor Port 3

        collectorLeft = hardwareMap.get(CRServo.class, "collectorLeft");                 //Control Hub Servo Port 0
        collectorRight = hardwareMap.get(CRServo.class, "collectorRight");               //Control Hub Servo Port 1

        armMotor   = hardwareMap.get(DcMotor.class, "arm"); //the arm motor              //Expansion Hub Motor Port 0
        slideMotor = hardwareMap.get(DcMotor.class, "slide");                            //Expansion Hub Motor Port 1

        slideButton = hardwareMap.get(TouchSensor.class, "slideButton");                 //Expansion Hub Sensor Port 0

        //odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");                        // Control Hub I2C Port 0


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

/*

         */
/*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         *//*

        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        */
/*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         *//*

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        */
/*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         *//*

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        */
/*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         *//*

        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
*/


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("X offset", odo.getXOffset());
        //telemetry.addData("Y offset", odo.getYOffset());
        //telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        //telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }
}
