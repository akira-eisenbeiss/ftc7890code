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

/*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
*/
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Autonomous(name="FULL AUTO FINAL", group="LinearOpMode")
public class FULL_AUTO extends LinearOpMode {

    //MOTORS
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftIntake;
    DcMotor rightIntake;
    DcMotor drawbridge;
    //SERVOS
    CRServo ballArm;
    //SENSORS
    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;
    ColorSensor jewelSensor; //attached to ballArm
    ColorSensor cryptoSensor;
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
    int targetCount = 3;
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
    private final static double move = 0.4;
        /*
        USED IN MOVEMENT METHODS
        */

    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;

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
        jewelSensor = hardwareMap.colorSensor.get("jewel sensor");
        cryptoSensor = hardwareMap.colorSensor.get("crypto sensor");
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) MRGyro;
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound range");
        //SERVOS
        ballArm = hardwareMap.crservo.get("ball arm");
/*
        //VUFORIA
        parameters.vuforiaLicenseKey = "AcoS+YP/////AAAAGTq922ywuU6FquBqcm2CeatGNf2voKamgXI1KwF7yLiQKP+RqBNrI4ND0i98TsuYnBytFG0YYUz2+4wvHBN5pz+/CacheTAG6upbc95Ts0UJgGRg0aTLaVzdYUQUI5dRlAh50DsGYdPkabTZmPO+5EYj79XDDHhok7wTZDb6ZyiCLlzXtM5EZ9nyiWQxz6XJ3M7Q+m4nVuaAdvWN+qwkQsqohSoxB8TNI4dDYlSMQbbO6d3SkCgfXy4K8y/lBNDF8suTeSgNY0YGs/N5FIYTLa+eyu+r3kbf2ig0EsL1Er+AhLZkVDpksvMp+MMBdDVyi6JDjr4E+P2D82ztt8Ex0aoR+h0n4RyRnkS+G4FB4wRD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
*/
        // relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();

        // relicTrackables.activate();

        while (opModeIsActive()) {
            jewel();
            dividerCount();
            scoreTurning();
            scoreGlyph();
            break;
        }
    }
    //HITS JEWEL
    public void jewel(){
        while(!sensed) {
            extendBallArm();
            if (isColor().equals("RED")){
                telemetry.addData("DETECTED COLOR", "RED");
                telemetry.update();
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(250);
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                sleep(5000);
                ballArm.setPower(-out);
                sleep(1000);
                sensed = true;
            }
            else if (isColor().equals("BLUE")) {
                telemetry.addData("DETECTED COLOR", "BLUE");
                telemetry.update();
                leftStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(250);
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                sleep(5000);
                ballArm.setPower(-out);
                sleep(1000);
                sensed = true;
            }
            else if (isColor().equals("SKIP")){
                telemetry.addData("DETECTED COLOR", "SKIP");
                telemetry.update();
                ballArm.setPower(-out);
                sensed = true;
            }
        }
    }
    //EXTENDS JEWEL ARM UNTIL IT SENSES JEWEL
    public void extendBallArm(){
        while (!stopArm) {
            if (jewelSensor.blue() > jewelSensor.red() || jewelSensor.red() > jewelSensor.blue()) {
                //unsure about this logic
                ballArm.setPower(0);
                stopArm = true;
            } else {
                ballArm.setPower(out); //TODO: fix this value!
                telemetry.addLine("extending ball arm");
                telemetry.update();
            }
        }
    }
    //DETECTS COLOR
    public String isColor(){
        if (jewelSensor.blue() > jewelSensor.red()) {
            telemetry.addLine("DETECTED BLUE");
            telemetry.update();
            return "BLUE";
        }
        else if (jewelSensor.blue() < jewelSensor.red()){
            telemetry.addLine("DETECTED RED");
            telemetry.update();
            return "RED";
        }
        else {
            telemetry.addLine("DETECTED NOTHING");
            telemetry.update();
            return "SKIP";
        }
    }
    //POSITIONS ROBOT AT CIPHER
    public void dividerCount(){
        telemetry.addLine("divider count activated");
        telemetry.update();
        /*
        while (forwards > 0) {
            if (rangeSensor.getDistance(DistanceUnit.CM) > 20 && forwards == 2) {
                //moving forward so that cryptoSensor can sense cipher
                moveForward(leftFront, leftBack, rightFront, rightBack);
                telemetry.addLine("moving forward");
                telemetry.update();
            }
            else if (forwards == 1) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(3000);
                telemetry.addLine("going right to clear balance board");
                telemetry.update();
                forwards = 2;
            }
            else {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                telemetry.addLine("arrived at distance");
                telemetry.update();
                forwards = 0;
            }
        }
        */
        while (targetCount > cntr) {
            rightStrafe(leftFront, leftBack, rightFront, rightBack);
            telemetry.addLine("strafing to cypher");
            telemetry.update();
            if (cryptoSensor.blue() > cryptoSensor.red()) {
                cntr++;
                sleep(50);
                cryptoCheck();
            }
        }
    }
    public void cryptoCheck(){
        if (targetCount == cntr){
            telemetry.addLine("at cipher");
            telemetry.update();
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
        leftIntake.setPower(0.7);
        rightIntake.setPower(-0.7);
    }

    //methods for basic movements
    public static void stopDatMovement(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    //strafe method. Once again, always put left motors first!!
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

    //move backwards method. Also, always put the left motors first, dumbo
    public void moveBackwards(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(move);
        motor3.setPower(-move);
        motor4.setPower(-move);
    }

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
