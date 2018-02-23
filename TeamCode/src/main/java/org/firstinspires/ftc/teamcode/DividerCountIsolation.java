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

@Autonomous(name="RENAME FILES ERIN  arm isolation", group="isolation")
public class DividerCountIsolation extends LinearOpMode{

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor leftIntake, rightIntake;

    CRServo ballArm;

    ColorSensor jewelSensorL;

    int cntr = 0;
    int targetCount = 1;
    double move = 0.5;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftIntake = hardwareMap.dcMotor.get("left intake");

        ballArm = hardwareMap.crservo.get("ball arm");

        jewelSensorL = hardwareMap.colorSensor.get("jewel sensor L");

        double out = 0.3;

        waitForStart();

        while (opModeIsActive()) {;
            while (targetCount > cntr) {
                rightStrafe(leftFront, leftBack, rightFront, rightBack);
                telemetry.addData("status", "strafing to cypher");
                telemetry.update();
                if (jewelSensorL.blue() > jewelSensorL.red()) {
                    telemetry.addData("status", "sensed crypto box");
                    cntr++;
                    sleep(50);
                }
                if (targetCount == cntr) {
                    stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                }

            }
            rightStrafe(leftFront, leftBack, rightFront, rightBack);
            sleep(3000);
            telemetry.addLine("got to cypher");
            telemetry.update();
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
}
