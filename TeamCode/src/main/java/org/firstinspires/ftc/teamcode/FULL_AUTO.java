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

import java.util.Locale;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="FULL AUTO FINAL", group="LinearOpMode")
public class FULL_AUTO extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

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
    /*
    THE DURATION OF THE BALLARM EXTENDING
    used TODO: ADD INFO ERIN
     */
    private final static double move = 0.4;
    /*
    SPEED OF THE WHEEL MOTORS
    used in all movement methods
    */

    //GYRO SENSOR CODES
    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;

    //ENCODERS
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    //VUFORIA
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException{

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

        /*
        Below are our actual methods for
        autonomous...
         */
        jewel();
        vuMark();
        dividerCount();
        scoreTurning();
        scoreGlyph();
    }

    //KNOCKS OFF THE CORRECT JEWEL
    public void jewel(){
        while(!sensed) {
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
        while (!stopArm) {
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
        telemetry.addLine("got to cypjer");
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
        while (!turned) {
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
        }
    }

    //VUFORIA METHOD
    public void vuMark(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZD9V+f/////AAAAGZDj5Sa0JkeSohNtCSdf0T94R9bz9UlzQyuCIZuJ2d1ehAEqmbPYzprSq4hmd/9XUZYT088pUIzwl79q9h2ljFjUFD5p0RHKBx+ggMJ+qgCelvbeNf7Rd771vlduzibSBN6np49m6Z31Eyk0dYFZJbpdmw4P7mQ8LaeR6UOLgmiythqcCZga9VoEHPA2e8Z9/7At1SZVPiOBkVlEKz5AGhPhL5/A/R3sb30NSaiq5yquyJ+sOWvNQ5ovdVND6OrAQrc2DdQcCDyD8JQLOiVZYCPoNohKfuZ9N2jnZRSueEH4XV6i2DOqWxfJ5vmNf6jBcrOWLROO8KEoPa2Fvibxj7lPMp4JM/nMXK7TwEopU91v";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
        boolean detectedPicto = false;
        RelicRecoveryVuMark vuMark;
        while(detectedPicto == false) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                detectedPicto  = true;
                stopDatMovement(leftFront,leftBack,rightFront,rightBack);
                telemetry.addData("Vuforia", vuMark);

            }
            else {
                telemetry.addData("Vuforia", "NOT DETECTED");
                leftStrafe(leftFront,leftBack,rightFront,rightBack);
                //TODO: FIX THE STRAFING DIRECTIONS!?
            }
            telemetry.update();
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