package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.*;

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


@Autonomous(name="autonomous full", group="Linear Opmode")
public class AutonomousFull extends LinearOpMode {

    //sorry about these strings, btw
    public final static String LEFTFRONT = "leftFront";
    public final static String LEFTBACK = "leftBack";
    public final static String RIGHTFRONT = "rightFront";
    public final static String RIGHTBACK = "rightBack";
    public final static String BALLARM = "ballArm";

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private ColorSensor color_sensor;
    private Servo ballArm;


    public boolean detected = false;
    @Override
    public void runOpMode() {
        color_sensor = hardwareMap.colorSensor.get("color");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double lfDrive;
        double lbDrive;
        double rfDrive;
        double rbDrive;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            leftFront = hardwareMap.get(DcMotor.class, LEFTFRONT);
            leftBack = hardwareMap.get(DcMotor.class, LEFTBACK);
            rightFront = hardwareMap.get(DcMotor.class, RIGHTFRONT);
            rightBack = hardwareMap.get(DcMotor.class, RIGHTBACK);
            ballArm = hardwareMap.get(Servo.class, BALLARM);

            double ballposition = 1;
            ballArm.setPosition(-1.0);
            double move = 0.5;
            color_sensor = hardwareMap.colorSensor.get("color");

            if(detected == false) {
                if (color_sensor.blue() > color_sensor.red()) {
                    leftFront.setPower(-move);
                    leftBack.setPower(-move);
                    rightFront.setPower(move);
                    rightBack.setPower(move);
                    sleep(1000);
                    stopDatMovement(leftFront, rightFront, leftBack, rightBack);
                    ballArm.setPosition(0.0);
                    detected = true;
                } else if (color_sensor.red() > color_sensor.blue()) {
                    leftFront.setPower(move);
                    leftBack.setPower(move);
                    rightFront.setPower(-move);
                    rightBack.setPower(-move);
                    sleep(1000);
                    stopDatMovement(leftFront, rightFront, leftBack, rightBack);
                    ballArm.setPosition(0.0);
                    detected = true;
                }
//bool
            }
            ballArm.setPosition(0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
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