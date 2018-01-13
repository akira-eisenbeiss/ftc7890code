package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name="autonomous blue relic side", group="Linear Opmode")
public class AutoBlueR extends LinearOpMode {

    //sorry about these strings, btw
    public final static String LEFTFRONT = "leftFront";
    public final static String LEFTBACK = "leftBack";
    public final static String RIGHTFRONT = "rightFront";
    public final static String RIGHTBACK = "rightBack";
    public final static String BALLARM = "ballArm";
    //intake wheels
    public final static String LEFTINTAKE = "leftIntake";
    public final static String RIGHTINTAKE = "rightIntake";
    //servoes outtake of glyphs
    //knocks off jewel
    private final static String MOVEJEWEL = "moveJewel";
    //gyro sensor stuff
    private final static String MRGYRO = "MRGyro";
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private ColorSensor color_sensor;
    private Servo ballArm;
    private Servo moveJewel;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    public final static double move = 0.5;
    public final static double slowMove = move / 2; //minor change
    public boolean detected = false;
    public boolean runner = false;
    public boolean turner = false;
    public boolean sensi = false;
    public boolean sense2 = false;
    private ColorSensor cryptoSensor;
    //gyro stuff
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro MRGyro;



    @Override
    public void runOpMode() {

        //gyro stuff for resetting Zheader
        boolean resetState = false;
        boolean lastResetState = false;
        //variable for how much we want the robo to turn
        int targetHeading = 90;

        color_sensor = hardwareMap.colorSensor.get("color");
        cryptoSensor = hardwareMap.colorSensor.get("color");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double lfDrive;
        double lbDrive;
        double rfDrive;
        double rbDrive;

        OpenGLMatrix lastLocation = null; //from 4326

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        leftFront = hardwareMap.get(DcMotor.class, LEFTFRONT);
        leftBack = hardwareMap.get(DcMotor.class, LEFTBACK);
        rightFront = hardwareMap.get(DcMotor.class, RIGHTFRONT);
        rightBack = hardwareMap.get(DcMotor.class, RIGHTBACK);
        ballArm = hardwareMap.get(Servo.class, BALLARM);
        moveJewel = hardwareMap.get(Servo.class, MOVEJEWEL);
        //intake wheels, hahahahahahaha
        leftIntake = hardwareMap.get(DcMotor.class, LEFTINTAKE);
        rightIntake = hardwareMap.get(DcMotor.class, RIGHTINTAKE);
        // run until the end of the match
        //calirbate gyro
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, MRGYRO);
        gyro = (IntegratingGyroscope) MRGyro;

        MRGyro.calibrate();


        while (opModeIsActive()) {

            double ballposition = 1;
            color_sensor = hardwareMap.colorSensor.get("color");
            cryptoSensor = hardwareMap.colorSensor.get("color");
            //for storing raw data
            //dunno if needed
            int rawX = MRGyro.rawX();
            int rawY = MRGyro.rawY();
            //heading variable

            moveJewel.setPosition(0.0);
            //all of this needs to be fixe
            if (!detected) {
                ballArm.setPosition(-1.0);
                //we need to change this code so that the arm moves, not the robot
                if (color_sensor.blue() < color_sensor.red()) {
                    //no. for testing purposes only
                    //make it so arm will move forward, hitting red jewel in front
                    moveJewel.setPosition(0);
                    //the problem coud beb/c
                    ballArm.setPosition(1.0);
                    detected = true;
                    runner = true;
                } else if (color_sensor.red() < color_sensor.blue()) {
                    //again, no. for testing purposes only
                    moveJewel.setPosition(0);
                    ballArm.setPosition(1.0);
                    detected = true;

                    runner = true;
                }
//bool
            }

            if (runner) {
                //hard code for moving off balancing stone
                moveBackwards(leftFront, leftBack, rightFront, rightBack);
                sleep(1000);
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);

                turner = true;
                telemetry.addData("STATUS", "moving to cryptobox");
                telemetry.update();
                runner = false;
            }

            //resets the z heading but only once when buttons pressed
            //don't actualy know if necessary
            resetState = (gamepad2.x && gamepad2.y);
            if (resetState && !lastResetState) {
                MRGyro.resetZAxisIntegrator();
            }
            lastResetState = resetState;
            int heading = MRGyro.getHeading();

            if (turner) {
                //x is the left-right direction if the wire is at the bottom
                heading = MRGyro.getHeading();
                while (heading < targetHeading ) {
                    //turns, hopefully
                    leftFront.setPower(0.5);
                    leftBack.setPower(0.5);
                    rightFront.setPower(0.5);
                    rightBack.setPower(0.5);

                    heading = MRGyro.getHeading();

                    sensi = true;
                    turner = false;
                }
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
            }


            if (sensi) {

                leftFront.setPower(0.7);
                leftBack.setPower(0.7);
                rightFront.setPower(0.7);
                rightBack.setPower(0.7);
                //rightStrafe(leftFront, leftBack, rightFront, rightBack);

                if (cryptoSensor.blue() > cryptoSensor.red() && sense2) {
                    stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                    //sleep is for testing and the weak
                    moveForward(leftFront, leftBack, rightFront, rightBack);
                    sleep(100);
                    //RELEASE YOUR GLYPH INTO THE BOX
                    leftIntake.setPower(-1.0);
                    rightIntake.setPower(-1.0);
                    //moves robot forward a little
                    //the sleep time is probs inacccurate

                    sensi = false;
                }

                else if (cryptoSensor.blue() > cryptoSensor.red()) {
                    sense2 = true;
                }
            }

        }
    }
    public static void stopDatMovement(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4)
    {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    //strafe method. Once again, always put left motors first!!
    public static void leftStrafe(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(-move);
        motor3.setPower(move);
        motor4.setPower(-move);
    }
    public static void rightStrafe(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-move);
        motor2.setPower(move);
        motor3.setPower(-move);
        motor4.setPower(move);
    }
    //move forward method
    public static void moveForward(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4)
    {
        motor1.setPower(-move);
        motor2.setPower(-move);
        motor3.setPower(move);
        motor4.setPower(move);
    }

    //move backwards method. Also, always put the left motors first, dumbo
    public static void moveBackwards(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(move);
        motor3.setPower(-move);
        motor4.setPower(-move);
    }

    public static void rotateCCW(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(move);
        motor3.setPower(move);
        motor4.setPower(move);
    }

}
