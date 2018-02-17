package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Autonomous(name="FULL AUTO FINAL", group="LinearOpMode")
public class FULL_AUTO extends LinearOpMode {

    //MOTORS
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor leftIntake, rightIntake;
    DcMotor drawbridge;

    //SERVOS
    CRServo ballArm;

    //SENSORS
    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;
    ColorSensor jewelSensorL, jewelSensorR; //these two are mounted on the ballArm
    OpticalDistanceSensor odsSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;

    //COUNTERS AND BOOLEANS
    boolean sensed = false;
    /*
    USED TO SEE IF THE BALLARM SHOULD KEEP GOING
    used in[jewel, extendBallArm]
     */
    boolean turned = false;
    /*
    USED TO MAKE SURE ROBOT CONTINUOSLY TURNS
    used in [scoreTurning]
     */
    boolean stopArm = false;
    /*
    MAKES SURE JEWEL ARM CONTINUES TO EXTEND UNTIL COLOR IS SENSED
    used in [extendBallArm]
    */
    boolean scoring = false;
    /*

     */
    int forwards = 1;
    /*
    USED TO MAKE SURE ROBOT MOVES TO RIGHT DISTANCE FROM WALL
    used in [dividerCount]
    */
    int cntr = 0;
    /*
    USED TO COUNT THE DIVIDERS THAT WE PASS
    used in [cryptoCheck, dividerCount]
     */
    int targetCount = 2;
    /*
    THE DESIRED AMOUNT OF DIVIDERS DETECTED
    used in [cryptoCheck, dividerCount]
     */
    //SPEEDS
    double out = 0.3;
    /*
    THE POWER SET TO MOVE THE BALLARM OUT AND IN
    used in[jewel, extendBallArm]
     */
    int extendArm = 4750;

    private final static double move = 0.4;
        /*
        USED IN MOVEMENT METHODS
        */

    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;

    @Override
    public void runOpMode() {
        //MOTORS
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");
        drawbridge = hardwareMap.dcMotor.get("drawbridge");
        //SENSORS
        jewelSensorL = hardwareMap.colorSensor.get("jewel sensor L");
        jewelSensorR = hardwareMap.colorSensor.get("jewel sensor R");
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) MRGyro;
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound range");
        //SERVOS
        ballArm = hardwareMap.crservo.get("ball arm");

        waitForStart();

        while (opModeIsActive()) {
            while (!stopArm) {
                if (jewelSensorL.red() > jewelSensorL.blue() || jewelSensorL.red() < jewelSensorL.blue() || jewelSensorR.red() > jewelSensorR.blue() || jewelSensorR.red() < jewelSensorR.blue()) {
                    ballArm.setPower(0.0);
                    stopArm = true;
                } else {
                    ballArm.setPower(out);
                }
            }


            if (jewelSensorL.red() > jewelSensorL.blue() || jewelSensorR.blue() < jewelSensorR.red()) {
                leftStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(250);
                break;
            } else if (jewelSensorL.red() < jewelSensorR.blue() || jewelSensorR.blue() < jewelSensorR.red()) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(250);
                break;
            }
        }
    }

        //methods for basic movements
    public static void stopDatMovement(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    //strafe method
    public void leftStrafe(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(-move);
        motor3.setPower(move);
        motor4.setPower(-move);
    }
    public void rightStrafe(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-move);
        motor2.setPower(move);
        motor3.setPower(-move);
        motor4.setPower(move);
    }
    //move forward method
    public void moveForward(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-move);
        motor2.setPower(-move);
        motor3.setPower(move);
        motor4.setPower(move);
    }
    //move backwards method
    public void moveBackwards(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(move);
        motor3.setPower(-move);
        motor4.setPower(-move);
    }
    //rotating methods
    public static void rotateCW(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-move);
        motor2.setPower(-move);
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