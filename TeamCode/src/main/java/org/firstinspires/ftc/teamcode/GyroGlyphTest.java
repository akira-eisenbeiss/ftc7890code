package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="gyro glyph test", group="sensor")
@Disabled
public class GyroGlyphTest extends LinearOpMode {
    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    int targetHeadingGlyph = 270;

    public void runOpMode() {
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) MRGyro;

        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        waitForStart();

        while (opModeIsActive()) {

            int heading = MRGyro.getHeading();

            leftFront.setPower(-0.5);
            leftBack.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightBack.setPower(-0.5);

            if (heading > targetHeadingGlyph - 10 && heading < targetHeadingGlyph + 10) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                sleep(5000);
            }
        }
    }
    public static void stopDatMovement(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4)
    {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}
