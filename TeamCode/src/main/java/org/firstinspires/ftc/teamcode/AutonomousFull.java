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


@TeleOp(name="autonomous full", group="Linear Opmode")

@Disabled
public class AutonomousFull extends LinearOpMode {

    //sorry about these strings, btw
    public final static String LEFTFRONT = "leftFront";
    public final static String LEFTBACK = "leftBack";
    public final static String RIGHTFRONT = "rightFront";
    public final static String RIGHTBACK = "rightBack";
    public final static String ballArm = "BALLARM";

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private ColorSensor color_sensor;
    private Servo ballArm;


    @Override
    public void runOpMode() {
        color_sensor = hardwareMap.colorSensor.get("color");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double lfDrive;
        double lbDrive;
        double rfDrive;
        double rbDrive;

        ballArm = hardwareMap.get(DcMotor.class, BALLARM);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            leftFront = hardwareMap.get(DcMotor.class, LEFTFRONT);
            leftBack = hardwareMap.get(DcMotor.class, LEFTBACK);
            rightFront = hardwareMap.get(DcMotor.class, RIGHTFRONT);
            rightBack = hardwareMap.get(DcMotor.class, RIGHTBACK);

            double ballposition = 0.1;
            ballArm.setPosition(ballposition);
            color_sensor = hardwareMap.colorSensor.get("color");

            double move = 1;

            if(color_sensor.blue() > color_sensor.red()){
                leftFront.setPower(-move);
                leftBack.setPower(-move);
                rightFront.setPower(move);
                rightBack.setPower(move);
            }
            else if(color_sensor.red() > color_sensor.blue()){
                leftFront.setPower(move);
                leftBack.setPower(move);
                rightFront.setPower(-move);
                rightBack.setPower(-move);
            }else{

            }
            ballArm.setPosition(0.0);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}