package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="extend ballarmtest", group="LinearOpMode")
public class ExtendBallArmTest extends LinearOpMode {

    //MOTORS
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor leftIntake, rightIntake;
    DcMotor drawbridge;

    //SERVOS
    CRServo ballArm;

    boolean stopArm = false;
    double out = 0.3;

    public void runOpMode() {
        ballArm = hardwareMap.crservo.get("ball arm");

        waitForStart();

        while (opModeIsActive()) {
            extendBallArm();
        }
    }

    //EXTENDS JEWEL ARM UNTIL IT SENSES JEWEL
    public void extendBallArm() {
        if (opModeIsActive()) {
            while (!stopArm) {
                ballArm.setPower(out); //TODO: fix this value!
                telemetry.addLine("extending ball arm");
                telemetry.update();
            }
        }
    }
}
