package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="complete tele op3", group="Tele Op")
public class TeleOpNew extends OpMode {

    int intakePower = 0;
    int intakeServo = 0;
    DcMotor leftFront = hardwareMap.dcMotor.get("left front");
    DcMotor leftBack = hardwareMap.dcMotor.get("left back");
    DcMotor rightFront = hardwareMap.dcMotor.get("right front");
    DcMotor rightBack = hardwareMap.dcMotor.get("right back");
    DcMotor leftIntake = hardwareMap.dcMotor.get("left intake");
    DcMotor rightIntake = hardwareMap.dcMotor.get("right intake");
    CRServo lInServo = hardwareMap.crservo.get("left intake servo");
    CRServo rInServo = hardwareMap.crservo.get("right intake servo");
    DcMotor drawbridge = hardwareMap.dcMotor.get("drawbridge");

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        float drive = gamepad1.left_stick_y;
        float turn = gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);
        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);

        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        boolean gamepad2X = gamepad2.x;//not in use
        boolean gamepad2Y = gamepad2.y;
        float leftStick2 = gamepad2.left_stick_y;

        //wheel
        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);
        //TODO: fix negs and pos
        drawbridge.setPower(-leftStick2/2);

        if (gamepad2A && intakePower == 0) // in
        {
            leftIntake.setPower(1.0);
            rightIntake.setPower(1.0);
            intakePower ^= 1;
        } else if (gamepad2A && intakePower == 1)// out
        {
            leftIntake.setPower(-1.0);
            rightIntake.setPower(-1.0);
            intakePower ^= 1;
        } else if (gamepad2B) {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        if (gamepad2Y && intakeServo == 0) {
            //negatives and positives are for testin
            lInServo.setPower(-1.0);
            rInServo.setPower(1.0);
            intakeServo ^= 1;
        } else if (gamepad2Y && intakeServo == 1) {
            lInServo.setPower(0);
            rInServo.setPower(0);
            intakeServo ^= 1;
        }

        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
