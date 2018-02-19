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

@Autonomous(name="ball arm isolation", group="isolation")
public class ballArmIsolation extends LinearOpMode{

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor leftIntake, rightIntake;

    CRServo ballArm;

    ColorSensor jewelSensorL;
    
    int cntr = 0;

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
        
        while (opModeIsActive()) {
            if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorL.blue() > jewelSensorL.red() || cntr == 20) {
                ballArm.setPower(0);
                break;
            } else {
                ballArm.setPower(out); //TODO: fix this value!
                cntr++;
                telemetry.addLine("extending ball arm");
                telemetry.update();
            }
        }
    }
}
