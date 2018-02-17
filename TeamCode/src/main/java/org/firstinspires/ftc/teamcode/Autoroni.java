package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="autoronitony", group="sssa")
public class Autoroni extends LinearOpMode {

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

    //VUFORIA
    /* VuforiaLocalizer.Parameters parameters;
     VuforiaLocalizer vuforia;
     VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
     VuforiaTrackable relicTemplate = relicTrackables.get(0);
    */


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
jewel();
        }
    }
    public void jewel(){
        if (opModeIsActive()) {
            while(!sensed) {
                extendBallArm();
                if (jewelSensorL.red() > jewelSensorL.blue()){
                    telemetry.addData("DETECTED COLOR", "RED");
                    telemetry.update();
   /*                 leftStrafe(leftFront, leftBack, rightFront, rightBack);
                    sleep(250);
                    stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                    sleep(5000);
                    ballArm.setPower(-out);
                    sleep(1000);
                    sensed = true;
                    */
                }
                else if (jewelSensorL.blue() > jewelSensorL.red() ) {
                    telemetry.addData("DETECTED COLOR", "BLUE");
                    telemetry.update();
  /*                  rightStrafe(leftFront, leftBack, rightFront, rightBack);
                    sleep(250);
                    stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                    sleep(5000);
                    ballArm.setPower(-out);
                    sleep(1000);
                    sensed = true;
                    */
                }
                /*
                else if (isColor().equals("SKIP")){
                    telemetry.addData("DETECTED COLOR", "SKIP");
                    telemetry.update();
/*                    ballArm.setPower(-out);
                    sensed = true;
                    */
                }
            }
        }
    //EXTENDS JEWEL ARM UNTIL IT SENSES JEWEL
    public void extendBallArm(){
        if(opModeIsActive()) {
            while (!stopArm) {
                if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorL.red() > jewelSensorL.blue() || jewelSensorR.red() > jewelSensorR.blue() || jewelSensorR.blue() > jewelSensorR.red() ) {
                    ballArm.setPower(0);
                    break;
                } else {
                    ballArm.setPower(out); //TODO: fix this value!
                    telemetry.addLine("extending ball arm");
                    telemetry.update();
                }
            }
        }
    }
    /*
    String colorDetected;
    public String isColor(){
        if (opModeIsActive()) {
            if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorR.blue() > jewelSensorR.red()) {
                telemetry.addLine("DETECTED BLUE");
                telemetry.update();
                sleep(3000);
                colorDetected = "BLUE";
            }
            else if (jewelSensorL.blue() < jewelSensorL.red() ||jewelSensorR.blue() < jewelSensorR.red()){
                telemetry.addLine("DETECTED RED");
                telemetry.update();
                sleep(3000);
                colorDetected = "RED";
            }
            else {
                telemetry.addLine("DETECTED NOTHING");
                telemetry.update();
            }
        }
        return colorDetected;
    }
*/
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
