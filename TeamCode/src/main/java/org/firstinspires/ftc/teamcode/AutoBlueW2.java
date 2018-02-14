//test target heading glyph for gyro

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

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="auto blue w2", group="LinearOpMode")
public class AutoBlueW2 extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftIntake;
    DcMotor rightIntake;
    DcMotor drawbridge;

    //servos
    CRServo ballArm;
    //sensors
    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;
    ColorSensor jewelSensor;
    ColorSensor cryptoSensor;
    OpticalDistanceSensor odsSensor;
    //vuforia

    boolean sensed = false;
    int detected = 0;
    int counter = 1;
    int targetCount;
    //int targetCount = 0;
    private final static double move = 0.5;
    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;
    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    VuforiaTrackable relicTemplate = relicTrackables.get(0);
    ModernRoboticsI2cRangeSensor rangeSensor;


    @Override
    public void runOpMode() {

        //motors
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");
        drawbridge = hardwareMap.dcMotor.get("drawbridge");

        //sensors
        jewelSensor = hardwareMap.colorSensor.get("jewel sensor");
        cryptoSensor = hardwareMap.colorSensor.get("crypto sensor");

        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) MRGyro;
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound range");
        //servos without an 'e'
        ballArm = hardwareMap.crservo.get("ball arm");

        //vuforia
        parameters.vuforiaLicenseKey = "AcoS+YP/////AAAAGTq922ywuU6FquBqcm2CeatGNf2voKamgXI1KwF7yLiQKP+RqBNrI4ND0i98TsuYnBytFG0YYUz2+4wvHBN5pz+/CacheTAG6upbc95Ts0UJgGRg0aTLaVzdYUQUI5dRlAh50DsGYdPkabTZmPO+5EYj79XDDHhok7wTZDb6ZyiCLlzXtM5EZ9nyiWQxz6XJ3M7Q+m4nVuaAdvWN+qwkQsqohSoxB8TNI4dDYlSMQbbO6d3SkCgfXy4K8y/lBNDF8suTeSgNY0YGs/N5FIYTLa+eyu+r3kbf2ig0EsL1Er+AhLZkVDpksvMp+MMBdDVyi6JDjr4E+P2D82ztt8Ex0aoR+h0n4RyRnkS+G4FB4wRD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            jewel(ballArm);

            moveToCipher();

            moveTestWithout();
        }
    }

    //methods for different parts of autonomous
    public void jewel(CRServo CRServo1) {
        CRServo1.setPower(0.3);
        if (jewelSensor.red() > jewelSensor.blue() || jewelSensor.blue() > jewelSensor.red()) {
            detected = 1;
        }
        //sleep for testing
        sleep(1000);
        if (detected == 1) {
            if (jewelSensor.blue() > jewelSensor.red()) {
                leftStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(balanceMove);
                detected = 2;
            }
            else if (jewelSensor.red() > jewelSensor.blue()) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(balanceMove);
                leftStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(balanceMove * 2);
                detected = 2;
            }
        }
        if (detected == 2) {
            CRServo1.setPower(-0.3);
            sleep(900);
            CRServo1.setPower(0.0);

            int heading = MRGyro.getHeading();

            leftFront.setPower(-0.5);
            leftBack.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightBack.setPower(-0.5);

            if (heading > targetHeading - 10 && heading < targetHeading + 10) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                sensed = true;
            }
        }
    }

/*
    public RelicRecoveryVuMark acquireVuMark(VuforiaTrackable relicTemplate) {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && !sensed){
            leftStrafe(leftFront, leftBack, rightFront, rightBack);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        return vuMark;
    }

    public void vumark() {
        RelicRecoveryVuMark vuMark = acquireVuMark(relicTemplate);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        switch (vuMark) {
            case RelicRecoveryVuMark.LEFT:
                targetCount = 2;
                sensed = true;
                break;
            case vuMark == RelicRecoveryVuMark.CENTER:
                targetCount = 3;
                sensed = true;
                break;
            case vuMark == RelicRecoveryVuMark.RIGHT:
                targetCount = 4;
                sensed = true;
                break;
        }

        while (targetCount > counter) {
            rightStrafe(leftFront, leftBack, rightFront, rightBack)
            if (cryptoSensor.blue() > cryptoSensor.red()) {
                counter ++;
            }
        }

        if (targetCount == counter){
            scoreGlyph(leftIntake, rightIntake);
        }
    }
    */

    public void scoreGlyph(DcMotor motor1, DcMotor motor2){
        /* we need to move back a little bit
         * otherwise the glyph will get stuck
         */
        moveBackwards(leftFront, leftBack, rightFront, rightBack);
        sleep(20);

        //turning stuff
        int heading = MRGyro.getHeading();

        leftFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightFront.setPower(-0.5);
        rightBack.setPower(-0.5);

        if (heading > targetHeadingGlyph - 10 && heading < targetHeadingGlyph + 10) {
            stopDatMovement(leftFront, leftBack, rightFront, rightBack);
            motor1.setPower(0.7);
            motor2.setPower(-0.7);
        }
    }

    public void moveTest(){
        moveForward(leftFront, leftBack, rightFront, rightBack);
        sleep(20);
        //this target count is for autonomous without vuforia
        targetCount = 3;
        while (targetCount > counter) {
            rightStrafe(leftFront, leftBack, rightFront, rightBack);
            if (cryptoSensor.blue() > cryptoSensor.red()) {
                counter ++;
            }
        }

        if (targetCount == counter){
            scoreGlyph(leftIntake, rightIntake);
        }
    }

    public void moveTestWithout() {
        moveForward(leftFront, leftBack, rightFront, rightBack);
        sleep(20);
        //this target count is for autonomous without vuforia
        targetCount = 3;
        while (targetCount > counter) {
            rightStrafe(leftFront, leftBack, rightFront, rightBack);
            if (odsSensor.getLightDetected() > 0.01 && odsSensor.getLightDetected() < 0.2) {
                counter ++;
            }
        }

        if (targetCount == counter){
            scoreGlyph(leftIntake, rightIntake);
        }
    }
    public void moveToCipher() {
        //if moving foerward / strafing is unsuccessful, it's probs this cm distance or the ods.getLightDetected() distance
        while (rangeSensor.getDistance(DistanceUnit.CM) > 5 && sensed) {
            moveForward(leftFront, leftBack, rightFront, rightBack);
        }
    }

    //methods for specific actions
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
