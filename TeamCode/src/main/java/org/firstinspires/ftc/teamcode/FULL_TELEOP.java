package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
7890 Space Lions 2018 "FULL TELEOP FINAL"
GOALS: depositing two glyphs at a time, balancing stone
 */

@TeleOp(name="FULL TELEOP FINAL", group="Tele Op")
public class FULL_TELEOP extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //MOTORS
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftIntake;
    DcMotor rightIntake;
    DcMotor drawbridge;

    //SPEEDS
    int intakePower = 0;
    double treadPower = -1;
    double treadPowerSlow = -17/23;

    //DIRECTIONS
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    //SERVOS (only really used to make sure we can fix autonomous-generated problems)
    CRServo ballArm;

    @Override
    public void init() {

        //HARDWARE MAP
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");
        drawbridge = hardwareMap.dcMotor.get("drawbridge");
        ballArm = hardwareMap.crservo.get("ball arm");

        //SETTING DIRECTIONS
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);
    }

    @Override
    public void loop() {

        //MOVEMENT FLOATS
        float drive;
        float turn;
        float strafe;

        //CONTROL INVERSION
        float rightTrigger1 = gamepad1.right_trigger;
        if (rightTrigger1 <= 0.4) {
            drive = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
        } else {
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;
        }

        //DRIVING
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        if(gamepad1.left_trigger > 0.3){ //SLOW DRIVING
            leftFront.setPower(lfDrive/3);
            leftBack.setPower(lbDrive/3);
            rightFront.setPower(rfDrive/3);
            rightBack.setPower(rbDrive/3);

        }else{ //NORMAL DRIVING
            leftFront.setPower(lfDrive);
            leftBack.setPower(lbDrive);
            rightFront.setPower(rfDrive);
            rightBack.setPower(rbDrive);
        }

        //DRAWBRIDGE
        float dbSpeed = gamepad2.left_stick_y / 2;
        float dbSpeedSlow = gamepad2.left_stick_y / 4;

        if (gamepad2.right_trigger >= 0.3) {
            drawbridge.setPower(dbSpeed);
        }
        drawbridge.setPower(dbSpeedSlow);

        //INTAKE CODE
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        if (gamepad2A && intakePower == 0) // in
        {
            leftIntake.setPower(treadPower);
            rightIntake.setPower(-treadPowerSlow);
            intakePower ^= 1;
        } else if (gamepad2A && intakePower == 1)// out
        {
            leftIntake.setPower(-treadPower);
            rightIntake.setPower(treadPowerSlow);
            intakePower ^= 1;
        } else if (gamepad2B) {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            intakePower = 0;
        }

        //SAFETY CODE FOR BALLARM
        if (gamepad2.left_trigger > 0.6) {
            ballArm.setPower(-0.6);
            /*
            This code is so that if our autonomous breaks
            and our ballArm is stuck outwards, we can
            pull it back in without touching the robot
            and getting penalties.
             */
        } else {
            ballArm.setPower(0);
        }

        // TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
