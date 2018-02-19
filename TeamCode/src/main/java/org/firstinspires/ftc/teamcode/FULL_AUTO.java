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

import java.util.zip.Adler32;


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
    public void runOpMode() throws InterruptedException {

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

        jewel();
        dividerCount();
        scoreTurning();
        scoreGlyph();
        extendBallArm();
    }


    //KNOCKS OFF THE CORRECT JEWEL
    public void jewel(){
        while(sensed == false && opModeIsActive()) {
            extendBallArm();
            String color = isColor();
            if (color.equals("RED")){
                telemetry.addData("DETECTED COLOR", "RED");
                telemetry.update();
                leftStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(250);
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                sleep(5000);
                ballArm.setPower(-out);
                sleep(1000);
                sensed = true;
                break;
            }
            else if (color.equals("BLUE")) {
                telemetry.addData("DETECTED COLOR", "BLUE");
                telemetry.update();
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(250);
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                sleep(5000);
                ballArm.setPower(-out);
                sleep(1000);
                sensed = true;
                break;
            }
            else if (color.equals("SKIP")){
                telemetry.addData("DETECTED COLOR", "SKIP");
                telemetry.update();
                ballArm.setPower(-out);
                sensed = true;
                break;
            }
            else{
                telemetry.addData("DETECTED COLOR", "NONE");
                telemetry.update();
                //This shouldn't happen...
            }
        }
    }

    //EXTENDS JEWEL ARM UNTIL IT SENSES JEWEL
    public void extendBallArm(){
        while (stopArm == false && opModeIsActive()) {
            if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorL.red() > jewelSensorL.blue() || jewelSensorR.red() > jewelSensorR.blue() || jewelSensorR.blue() > jewelSensorR.red() ) {
                ballArm.setPower(0);
                stopArm = true;
                break;
            } else {
                ballArm.setPower(out); //TODO: fix this value!
                telemetry.addLine("extending ball arm");
                telemetry.update();
            }
        }
    }

    //DETECTS COLOR
    String colorDetected;
    public String isColor(){
        if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorR.blue() > jewelSensorR.red()) {
            telemetry.addLine("DETECTED BLUE");
            telemetry.update();
            colorDetected = "BLUE";
        }
        else if (jewelSensorL.blue() < jewelSensorL.red() ||jewelSensorR.blue() < jewelSensorR.red()){
            telemetry.addLine("DETECTED RED");
            telemetry.update();
            colorDetected = "RED";
        }
        else {
            telemetry.addLine("DETECTED NOTHING");
            telemetry.update();
            colorDetected = "SKIP";
        }
        return colorDetected;
    }
    //POSITIONS ROBOT AT CIPHER
    public void dividerCount(){
        telemetry.addLine("divider count activated");
        telemetry.update();
        stopDatMovement(leftFront, leftBack, rightFront, rightBack);
        sleep(5000);
        ballArm.setPower(0.5);
        sleep(extendArm);
        while (targetCount > cntr) {
            rightStrafe(leftFront, leftBack, rightFront, rightBack);
            telemetry.addLine("strafing to cypher");
            telemetry.update();
            if (cntr == 1) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                ballArm.setPower(0.5);
                sleep(extendArm);
            }
            else if (jewelSensorR.blue() > jewelSensorR.red()) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                ballArm.setPower(-0.5);
                sleep(extendArm);
                cntr++;
                sleep(50);
                cryptoCheck();
            }
        }
        rightStrafe(leftFront, leftBack, rightFront, rightBack);
        sleep(3000);
        telemetry.addLine("got to cypher");
        telemetry.update();

    }
    //DETERMINES POSITION CRYPTOBOX
    public void cryptoCheck(){
        if (targetCount == cntr){
            telemetry.addLine("at cipher");
            telemetry.update();
            ballArm.setPower(-0.5);
            sleep(extendArm);
            stopDatMovement(leftFront, leftBack, rightFront, rightBack);
        }
    }
    //TURNS ROBOT AT CIPHER
    public void scoreTurning() {
        while (turned == false && opModeIsActive()) {
            int heading = MRGyro.getHeading();

            leftFront.setPower(-0.5);
            leftBack.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightBack.setPower(-0.5);
            telemetry.addLine("turning");
            telemetry.update();

            if (heading > targetHeadingGlyph - 10 && heading < targetHeadingGlyph + 10) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                turned = true;
                telemetry.addLine("turned and ready");
                telemetry.update();
                sleep(1000);
                break;
            }
        }
    }
    //SCORES GLYPH
    public void scoreGlyph(){
        while(!scoring) {
            telemetry.addLine("outtaking glyph");
            telemetry.update();
            leftIntake.setPower(0.7);
            rightIntake.setPower(-0.7);
            break;
        }
    }

    //---------------------------//


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
