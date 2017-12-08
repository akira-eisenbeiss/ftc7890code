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

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
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

    //vuforia
    VuforiaLocalizer vuforia;

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
    public final static double move = 0.5;
    public final static double slowMove = 0.25;
    public boolean detected = false;
    @Override
    public void runOpMode() {

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AcoS+YP/////AAAAGTq922ywuU6FquBqcm2CeatGNf2voKamgXI1KwF7yLiQKP+RqBNrI4ND0i98TsuYnBytFG0YYUz2+4wvHBN5pz+/CacheTAG6upbc95Ts0UJgGRg0aTLaVzdYUQUI5dRlAh50DsGYdPkabTZmPO+5EYj79XDDHhok7wTZDb6ZyiCLlzXtM5EZ9nyiWQxz6XJ3M7Q+m4nVuaAdvWN+qwkQsqohSoxB8TNI4dDYlSMQbbO6d3SkCgfXy4K8y/lBNDF8suTeSgNY0YGs/N5FIYTLa+eyu+r3kbf2ig0EsL1Er+AhLZkVDpksvMp+MMBdDVyi6JDjr4E+P2D82ztt8Ex0aoR+h0n4RyRnkS+G4FB4wRD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrack = this.vuforia.loadTrackablesFromAsset("relicTrack");
        VuforiaTrackable Template = relicTrack.get(0);
        */

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

        leftFront = hardwareMap.get(DcMotor.class, LEFTFRONT);
        leftBack = hardwareMap.get(DcMotor.class, LEFTBACK);
        rightFront = hardwareMap.get(DcMotor.class, RIGHTFRONT);
        rightBack = hardwareMap.get(DcMotor.class, RIGHTBACK);
        ballArm = hardwareMap.get(Servo.class, BALLARM);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double ballposition = 1;
            ballArm.setPosition(-1.0);
            color_sensor = hardwareMap.colorSensor.get("color");

            if(detected == false) {
                if (color_sensor.blue() > color_sensor.red()) {
                    leftFront.setPower(-move);
                    leftBack.setPower(-move);
                    rightFront.setPower(move);
                    rightBack.setPower(move);
                    sleep(600);
                    stopDatMovement(leftFront, rightFront, leftBack, rightBack);
                    ballArm.setPosition(0.0);
                    detected = true;

                    //moves to the corner
                    leftFront.setPower(-move);
                    leftBack.setPower(move);
                    rightFront.setPower(-move);
                    rightBack.setPower(move);

                } else if (color_sensor.red() > color_sensor.blue()) {
                    sleep(700);
                    stopDatMovement(leftFront, rightFront, leftBack, rightBack);
                    ballArm.setPosition(0.0);
                    detected = true;


                }
//bool

            }


            ballArm.setPosition(0);

            /*
            sleep(2000);
            relicTrack.activate();
            boolean canSee = false;

            RelicRecoveryVuMark vuMark;

            while(canSee == false) {
                vuMark = RelicRecoveryVuMark.from(Template);
                telemetry.addData("VuMark", "%s visible", vuMark);

                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    canSee = true;
                    telemetry.addData("Vumark", vuMark);
                    leftFront.setPower(0.0);
                    leftBack.setPower(0.0);
                    rightFront.setPower(0.0);
                    rightBack.setPower(0.0);

                }else {
                    telemetry.addData("VuMark", "not visible");
                    //add some stuff in here

                    leftFront.setPower(slowMove);
                    leftBack.setPower(slowMove);
                    rightFront.setPower(-slowMove);
                    rightBack.setPower(-slowMove);
                }

                telemetry.update();
            }
        */
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)");
        telemetry.update();
    }

    public static void stopDatMovement(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4)
    {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

}