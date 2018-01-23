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


@Autonomous(name="auto blue r 3", group="LinearOpMode")
public class AutoBlueR2 extends LinearOpMode {

    //motors
    DcMotor leftFront = hardwareMap.dcMotor.get("left front");
    DcMotor leftBack = hardwareMap.dcMotor.get("left back");
    DcMotor rightFront = hardwareMap.dcMotor.get("right front");
    DcMotor rightBack = hardwareMap.dcMotor.get("right back");
    DcMotor leftIntake = hardwareMap.dcMotor.get("left intake");
    DcMotor rightIntake = hardwareMap.dcMotor.get("right intake");
    //servoes
    CRServo lInServo = hardwareMap.crservo.get("left intake servo");
    CRServo rInServo = hardwareMap.crservo.get("right intake servo");
    CRServo ballArm = hardwareMap.crservo.get("ball arm");
    //sensors
    ModernRoboticsI2cGyro MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    IntegratingGyroscope gyro = (IntegratingGyroscope) MRGyro;
    ColorSensor jewelSensorL = hardwareMap.colorSensor.get("ball sensor left");
    ColorSensor jewelSensorR = hardwareMap.colorSensor.get("ball sensor right");
    ColorSensor cryptoSensor = hardwareMap.colorSensor.get("crypto sensor");
    //vuforia 
  
    private final static double move = 0.5;
    int balanceMove = 250;
    int targetHeading = 270;
    VuforiaLocalizer.Parameters parameters;

    @Override
    public void runOpMode() {
        parameters.vuforiaLicenseKey = "AcoS+YP/////AAAAGTq922ywuU6FquBqcm2CeatGNf2voKamgXI1KwF7yLiQKP+RqBNrI4ND0i98TsuYnBytFG0YYUz2+4wvHBN5pz+/CacheTAG6upbc95Ts0UJgGRg0aTLaVzdYUQUI5dRlAh50DsGYdPkabTZmPO+5EYj79XDDHhok7wTZDb6ZyiCLlzXtM5EZ9nyiWQxz6XJ3M7Q+m4nVuaAdvWN+qwkQsqohSoxB8TNI4dDYlSMQbbO6d3SkCgfXy4K8y/lBNDF8suTeSgNY0YGs/N5FIYTLa+eyu+r3kbf2ig0EsL1Er+AhLZkVDpksvMp+MMBdDVyi6JDjr4E+P2D82ztt8Ex0aoR+h0n4RyRnkS+G4FB4wRD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
      
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        
        waitForStart();
      
        MRGyro.calibrate;
        relicTrackables.activate;
    }
  
    //methods for different parts of autonomous
    public static void jewel() {
        ballArm.setPower(0.3);
      //sleep for testing
        sleep(1000);
        if (jewelSensorL.red() > jewelSensorL.blue() || jewelSensorR.blue() > jewelSensorR.red()) {
            leftStrafe(leftFront, leftBack, rightFront, rightBack);
            sleep(balanceMove * 2);
        }
        else if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorR.red() > jewelSensorR.blue()) {
            rightStrafe(leftFront, leftBack, rightFront, rightBack);
            sleep(balanceMove);
            leftStrafe(leftFront, leftBack, rightFront, rightBack);
            sleep(balanceMove);
        }
    }
    
    public static void vumark() {
        
        
    } 
    public static void scoreGlyph(Dcmotor motor1){
        motor.setPower(-1.0);
        /* we need to move back a little bit
         * otherwise the glyph will get stuck
         */
        moveBackwards(leftFront, leftBack, rightFront, rightBack);
        sleep(20);
        stopDatMovement(leftFront, leftBack, rightFront, rightBack);
        
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
