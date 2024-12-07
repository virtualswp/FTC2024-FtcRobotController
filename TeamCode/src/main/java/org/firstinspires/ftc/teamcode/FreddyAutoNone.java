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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//Imports from auto example code
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Imports from FreddyTeleop
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;





@Autonomous(name="Freddy Auto - Move Right", group="Robot")
//@Disabled
public class FreddyAutoNone extends LinearOpMode {

    /* Declare hardware variables */
    public DcMotor leftFrontDriveMotor = null; //the left drivetrain motor
    public DcMotor rightFrontDriveMotor = null; //the right drivetrain motor
    public DcMotor leftRearDriveMotor = null; //the left drivetrain motor
    public DcMotor rightRearDriveMotor = null; //the right drivetrain motor

    public DcMotor armMotorLeft = null; //the arm motor

    public DcMotor armMotorRight = null;

    public DcMotor  slideMotor = null;

    private CRServo gripperWrist = null;
    private Servo gripperHand = null;

    private TouchSensor armButton = null;      // The Viper Slide button at the top of the clip


    // Member variables
    private FreddyAutoNone.armPosition currentArmPosition = FreddyAutoNone.armPosition.home;         //The current arm position
    private FreddyAutoNone.armPosition targetArmPosition = FreddyAutoNone.armPosition.home;          //The target arm position
    private int basketArmMoveStep = 0;                                      //While moving the arm in, what stage it's in

    private FreddyAutoNone.driveMode currentDriveMode = FreddyAutoNone.driveMode.collection;              //The current drive mode
    private boolean isArmButtonPressed = false;                           //Variable to determine if the slide button is being pressed


    private static final int SLIDE_LOW_BASKET  = 2500;                  //The degrees the encoder needs to move the slide motor to get to the low basket
    private static final int SLIDE_HIGH_BASKET = 5000;                  //The degrees the encoder needs to move the slide motor to get to the high basket



    private TouchSensor slideButton = null;      // The Viper Slide button at the top of the clip


    // Member variables
    private ElapsedTime runtime = new ElapsedTime();




    //Enumerations
    private enum armPosition {
        home,
        collectUp,
        collectDown,
        lowBasket,
        highBasket,
        endgame
    }

    private enum driveMode {
        normal,
        collection,
        deposit
    }



    @Override
    public void runOpMode() {
        this.ConfigureHardware();

        // Wait for the game to start (driver presses START)
        waitForStart();

        this.DriveReverseForTime(2.0);
        this.DriveStop();
        this.MoveArmToDownPosition(0.2, 0.2);

        sleep(5000);
    }



    private void MoveArmToDownPosition(double leftMotorPower, double rightMotorPower){
        /* This function will move the arm to the down position with a set power / speed */

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 10)) {
            this.HandleArmSensors();

            telemetry.addData("Touch Sensor Pressed: ", this.isArmButtonPressed);
            telemetry.update();

            armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ((DcMotorEx) armMotorLeft).setPower(leftMotorPower);
            ((DcMotorEx) armMotorRight).setPower(rightMotorPower);

            if (this.isArmButtonPressed) {
                ((DcMotorEx) armMotorLeft).setPower(0.0);
                ((DcMotorEx) armMotorRight).setPower(0.0);

                armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                armMotorLeft.setTargetPosition(0);
                armMotorRight.setTargetPosition(0);

                armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                this.currentArmPosition = FreddyAutoNone.armPosition.collectDown;

                break;
            }
        }
    }

    private void DriveReverseForTime(double Seconds){
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0.0;
        double rightPower = 0.0;

        // Tank Mode uses one stick to control each wheel.
        float driveSpeed = 0.4F;
        float driveOffset = 0.12F;
        leftPower  = driveSpeed;
        rightPower = -driveSpeed;

        leftRearDriveMotor.setPower(leftPower - driveOffset);
        leftFrontDriveMotor.setPower(leftPower);
        rightRearDriveMotor.setPower(rightPower);
        rightFrontDriveMotor.setPower(rightPower - driveOffset);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < Seconds)) {
            telemetry.addData("Path", "1-Forward: %4.1f S Elapsed", runtime.seconds());
            telemetry.addData("Run Time", runtime.seconds());

            telemetry.update();
        }
    }


    private void HandleArmSensors(){
        this.isArmButtonPressed = armButton.isPressed();
    }



    public void DriveStrafeRight(double Seconds) {
        ChassisMotorValues result = new ChassisMotorValues();

        double strafePower = 0.3;
        result.leftFront = -strafePower;    //Should move backwards
        result.rightFront = -strafePower;    //Should move forwards
        result.leftRear = strafePower;     //Should move forwards
        result.rightRear = strafePower;     //Should move backwards

        runtime.reset();

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

        gripperWrist = hardwareMap.get(CRServo.class, "gripperWrist");                   //Control Hub Servo Port 0
        gripperHand = hardwareMap.get(Servo.class, "gripperHand");                       //Control Hub Servo Port 1

        armMotorLeft = hardwareMap.get(DcMotor.class, "armleft");                        //Expansion Hub Motor Port 1
        armMotorRight = hardwareMap.get(DcMotor.class, "armright");//the arm motor       //Expansion Hub Motor Port 2
        slideMotor = hardwareMap.get(DcMotor.class, "slide");                            //Expansion Hub Motor Port 0

        armButton = hardwareMap.get(TouchSensor.class, "armButton");                     //Expansion Hub Sensor Port 0


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);

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

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotorLeft).setCurrentAlert(5,CurrentUnit.AMPS);
        ((DcMotorEx) armMotorRight).setCurrentAlert(5,CurrentUnit.AMPS);
        ((DcMotorEx) slideMotor).setCurrentAlert(5,CurrentUnit.AMPS);


        //Set the arm motor's motor encoder to 0
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set the viper slide motor encode to 0
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Is Ready.");
        telemetry.update();
    }

}
