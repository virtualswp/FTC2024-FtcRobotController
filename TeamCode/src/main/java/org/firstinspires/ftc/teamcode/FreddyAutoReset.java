package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//Imports from auto example code
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;



@Autonomous(name="Pre-Match", group="Robot")
//@Disabled
public class FreddyAutoReset extends LinearOpMode {


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

    private boolean isArmFrontButtonPressed = false;                     //Variable to determine if the front arm button is being pressed

    private boolean isArmBackButtonPressed = false;                     //Variable to determine if the back arm button is being pressed

    private boolean isArmSlideSwitchPressed = false;                    //Variable to determine if the slide's magnetic switch sensor is pressed

    private boolean isArmSlideBackButtonPressed = false;                //Variable to determine if the slide arm's back button / touch sensor is pressed


    //</editor-fold>

    //<editor-fold desc="Constants">
    private static final int SLIDE_LOW_BASKET = 1500;                  //The degrees the encoder needs to move the slide motor to get to the low basket
    private static final int SLIDE_HIGH_BASKET = 2900;                  //The degrees the encoder needs to move the slide motor to get to the high basket

    //Napolean = 450, Freddy = 550
    private static final int SLIDE_COLLECT_OUT = 750;                   //The degrees the encoder needs to move the slide motor to get to the collect out position

    private static final int SLIDE_HOME_RESET = -100;                   // The position to move the slide motor past the home (0) position to account for any variance with the encoder

    private static final int ARM_COLLECT_UP = -500;                     //The degrees to move the arm up slightly to get over the bar
    private static final int ARM_COLLECT_DELIVERY = -3700;             //The degrees to move the arm straight up to the delivery up position

    private static final int ARM_HOME_RESET = 400;                     // The position to move the arm motor past the home (0) position to account for any variance with the encoder

    private static final double HAND_OPEN_POSITION = 0.0;               // The servo position for the hand to be fully open.

    //Napoleon = 0.57, Freddy = 0.64
    private static final double HAND_CLOSED_POSITION = 0.64;            // The servo position for the hand to be fully closed.

    // Napolean = 0.70, Freddy = 0.67
    private static final double WRIST_DOWN_POSITION = 0.67;             // The servo position for the wrist to be fully down.

    private static final double WRIST_BACK_POSITION = 0.0;              // The servo position for the wrist to be fully back.

    private static final double WRIST_DELIVERY_POSITION = 0.30;         // The servo position for the wrist when delivering to the baskets

    private static final double WRIST_COLLECT_UP_POSITION = 0.5;        // The servo position for the wrist when moving to collect up position (slightly back to go over the bar)


    //</editor-fold>

    //<editor-fold desc="Op Mode & Handlers">

    @Override
    public void runOpMode() {
        //Configure the hardware
        this.ConfigureHardware();

        /* Wait for the game driver to press play */
        waitForStart();


        this.HandleArmSensors();
        this.SetHardwareDefaultPositions();


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

            //Check if everything is reset
            if (this.isArmFrontButtonPressed && this.isArmSlideBackButtonPressed){
                hardwareReset = true;
            }
        }

        sleep(5000);
    }

    private void HandleArmSensors() {
        this.isArmFrontButtonPressed = armButtonFront.isPressed();
        this.isArmBackButtonPressed = armButtonRear.isPressed();
        this.isArmSlideSwitchPressed = armSlideSwitch.isPressed();
        this.isArmSlideBackButtonPressed = armSlideButtonRear.isPressed();
    }

    //</editor-fold>

    //<editor-fold desc="Utility Methods">

    private void SetHardwareDefaultPositions() {
        /* This method will set any hardware default positions */

        //Set the hand gripper to an open position to start
        this.gripperHand.setPosition(HAND_CLOSED_POSITION);                //0.0 = All the way open, 1.0 is all the way closed.
        this.gripperWrist.setPosition(WRIST_BACK_POSITION);                //0.0 = All the way down, 1.0 is all the way back.
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


        //Set the arm motor to not use the encoder
        slideArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set the slide motor to not use the encoder
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Is Ready.");
        telemetry.update();
    }

    //</editor-fold>

}