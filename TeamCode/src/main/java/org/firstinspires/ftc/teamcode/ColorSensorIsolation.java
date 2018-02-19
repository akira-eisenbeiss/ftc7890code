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

@Autonomous(name="color sensor isolation", group="isolation")
public class ColorSensorIsolation extends LinearOpMode{

    ColorSensor jewelSensorL;

    @Override
    public void runOpMode() {
        jewelSensorL = hardwareMap.colorSensor.get("jewel sensor L");

        while (opModeIsActive()) {
            String color = isColor();
            if (color.equals("RED")) {
                telemetry.addData("DETECTED COLOR", "RED");
                telemetry.update();
            } else if (color.equals("BLUE")) {
                telemetry.addData("DETECTED COLOR", "BLUE");
                telemetry.update();
            } else if (color.equals("SKIP")) {
                telemetry.addData("DETECTED COLOR", "SKIP");
                telemetry.update();
            } else {
                telemetry.addData("DETECTED COLOR", "NONE");
                telemetry.update();
                //This shouldn't happen...
            }
        }
    }
    String colorDetected;
    public String isColor() {
        if (jewelSensorL.blue() > jewelSensorL.red()) {
            telemetry.addLine("DETECTED BLUE");
            telemetry.update();
            colorDetected = "BLUE";
        }
        else if (jewelSensorL.blue() < jewelSensorL.red()){
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
}
