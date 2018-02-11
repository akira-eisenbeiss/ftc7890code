package org.firstinspires.ftc.teamcode;

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


//@Autonomous(name="auto blue r2", group="LinearOpMode")
@Disabled
public class AutoBlueR2 extends LinearOpMode {

    //motors
    DcMotor leftFront = hardwareMap.dcMotor.get("left front");
    DcMotor leftBack = hardwareMap.dcMotor.get("left back");
    DcMotor rightFront = hardwareMap.dcMotor.get("right front");
    DcMotor rightBack = hardwareMap.dcMotor.get("right back");
    DcMotor leftIntake = hardwareMap.dcMotor.get("left intake");
    DcMotor rightIntake = hardwareMap.dcMotor.get("right intake");
    DcMotor drawbridge = hardwareMap.dcMotor.get("drawbridge");
    
    //servoes
    CRServo ballArm = hardwareMap.crservo.get("ball arm");
    //sensors
    ModernRoboticsI2cGyro MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    IntegratingGyroscope gyro = (IntegratingGyroscope) MRGyro;
    ColorSensor jewelSensorL = hardwareMap.colorSensor.get("ball sensor left");
    ColorSensor jewelSensorR = hardwareMap.colorSensor.get("ball sensor right");
    ColorSensor cryptoSensor = hardwareMap.colorSensor.get("crypto sensor");
    //vuforia

    boolean sensed = false;
    boolean detected = false;
    int counter = 1;
    private final static double move = 0.5;
    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;
    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    VuforiaTrackable relicTemplate = relicTrackables.get(0);

    @Override
    public void runOpMode() {
        parameters.vuforiaLicenseKey = "AcoS+YP/////AAAAGTq922ywuU6FquBqcm2CeatGNf2voKamgXI1KwF7yLiQKP+RqBNrI4ND0i98TsuYnBytFG0YYUz2+4wvHBN5pz+/CacheTAG6upbc95Ts0UJgGRg0aTLaVzdYUQUI5dRlAh50DsGYdPkabTZmPO+5EYj79XDDHhok7wTZDb6ZyiCLlzXtM5EZ9nyiWQxz6XJ3M7Q+m4nVuaAdvWN+qwkQsqohSoxB8TNI4dDYlSMQbbO6d3SkCgfXy4K8y/lBNDF8suTeSgNY0YGs/N5FIYTLa+eyu+r3kbf2ig0EsL1Er+AhLZkVDpksvMp+MMBdDVyi6JDjr4E+P2D82ztt8Ex0aoR+h0n4RyRnkS+G4FB4wRD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();

        MRGyro.calibrate();
        relicTrackables.activate();

        while (opModeIsActive()) {

            jewel(ballArm);

            vumark();
        }
    }

    //methods for different parts of autonomous
    public void jewel(CRServo CRServo1) {
        CRServo1.setPower(0.3);
        if (jewelSensorL.red() > jewelSensorL.blue() || jewelSensorR.blue() > jewelSensorR.red() || jewelSensorL.blue() > jewelSensorL.red() || jewelSensorR.red() > jewelSensorR.blue()) {
            CRServo1.setPower(0.1);
            sleep(100);
            CRServo1.setPower(0.0);
            sleep(1000);
            detected = true;
        }
        //sleep for testing
        sleep(1000);
        if (detected) {
            if (jewelSensorL.red() > jewelSensorL.blue() || jewelSensorR.blue() > jewelSensorR.red()) {
                leftStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(balanceMove * 2);
                detected = false;
            }
            else if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorR.red() > jewelSensorR.blue()) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(balanceMove);
                leftStrafe(leftFront, leftBack, rightFront, rightBack);
                sleep(balanceMove);
                detected = false;
            }
            CRServo1.setPower(-0.5);
            sleep(900);
            CRServo1.setPower(0.0);
        }
    }


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

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            sensed = true;

            if (cryptoSensor.blue() > cryptoSensor.red() && counter == 1) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                scoreGlyph(leftIntake, rightIntake);
                counter += 3;
            }

            else if (counter == 1) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
            }

        }

        if (vuMark == RelicRecoveryVuMark.CENTER) {
            sensed = true;

            if (cryptoSensor.blue() > cryptoSensor.red() && counter == 1) {
                counter++;
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
            }

            else if (cryptoSensor.blue() > cryptoSensor.red() && counter == 2) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                scoreGlyph(leftIntake, rightIntake);
                counter += 2;
            }

            else if (counter == 1) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
            }
        }

        if (vuMark == RelicRecoveryVuMark.RIGHT){
            sensed = true;

            if (cryptoSensor.blue() > cryptoSensor.red() && counter == 1) {
                counter ++;
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
            }

            else if (cryptoSensor.blue() > cryptoSensor.red() && counter == 2) {
                counter ++;
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
            }

            else if (cryptoSensor.blue() > cryptoSensor.red() && counter == 3) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                scoreGlyph(leftIntake, rightIntake);
            }

            else if (counter == 1) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
            }
        }
    }

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
