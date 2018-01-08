package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name="autonomous full red", group="Linear Opmode")
public class AutonomousFullRed extends LinearOpMode {

    //vuforia
    VuforiaLocalizer vuforia;    //vuforia initializer

    //sorry about these strings, btw
    public final static String LEFTFRONT = "leftFront";
    public final static String LEFTBACK = "leftBack";
    public final static String RIGHTFRONT = "rightFront";
    public final static String RIGHTBACK = "rightBack";
    public final static String BALLARM = "ballArm";
<<<<<<< HEAD
    private ColorSensor testy_color_sensor;
=======
    //outtake of glyphs
    public Servo leftOut;
    public Servo rightOut;
>>>>>>> 0aaa1179eab32ca6ed08583fd62c35a08fceb285
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private ColorSensor color_sensor;
    private Servo ballArm;
    public final static double move = 0.5;
    public final static double slowMove = move / 2; //minor change
    public boolean detected = false;
    //servoes for outtake of glyphs

    @Override
    public void runOpMode() {

        /*
        //Vuforia - starting the camera  --> from 4326 code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //License Key, I already set it up I think
        parameters.vuforiaLicenseKey = "AZD9V+f/////AAAAGZDj5Sa0JkeSohNtCSdf0T94R9bz9UlzQyuCIZuJ2d1ehAEqmbPYzprSq4hmd/9XUZYT088pUIzwl79q9h2ljFjUFD5p0RHKBx+ggMJ+qgCelvbeNf7Rd771vlduzibSBN6np49m6Z31Eyk0dYFZJbpdmw4P7mQ8LaeR6UOLgmiythqcCZga9VoEHPA2e8Z9/7At1SZVPiOBkVlEKz5AGhPhL5/A/R3sb30NSaiq5yquyJ+sOWvNQ5ovdVND6OrAQrc2DdQcCDyD8JQLOiVZYCPoNohKfuZ9N2jnZRSueEH4XV6i2DOqWxfJ5vmNf6jBcrOWLROO8KEoPa2Fvibxj7lPMp4JM/nMXK7TwEopU91v";
        //Which camera we choose - I used back right now because I think the range is better, but front could work
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //Define Pictographs as VuMarks which the Vuforia can track
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
*/
        testy_color_sensor = hardwareMap.colorSensor.get("color");
        color_sensor = hardwareMap.colorSensor.get("color");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double lfDrive;
        double lbDrive;
        double rfDrive;
        double rbDrive;

//probs remove
        OpenGLMatrix lastLocation = null; //from 4326

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        leftFront = hardwareMap.get(DcMotor.class, LEFTFRONT);
        leftBack = hardwareMap.get(DcMotor.class, LEFTBACK);
        rightFront = hardwareMap.get(DcMotor.class, RIGHTFRONT);
        rightBack = hardwareMap.get(DcMotor.class, RIGHTBACK);
        ballArm = hardwareMap.get(Servo.class, BALLARM);
        //servo for outtakes
        leftOut = hardwareMap.servo.get("left out");
        rightOut = hardwareMap.servo.get("right out");
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

/*
            double ballposition = 1;
            color_sensor = hardwareMap.colorSensor.get("color");

            if(!detected) {
                ballArm.setPosition(-1.0);
                if (color_sensor.blue() < color_sensor.red()) {
                    leftFront.setPower(-move);
                    leftBack.setPower(-move);
                    rightFront.setPower(move);
                    rightBack.setPower(move);
                    sleep(600);
                    stopDatMovement(leftFront, rightFront, leftBack, rightBack);
                    sleep(2000);
                    ballArm.setPosition(1.0);
                    detected = true;

                } else if (color_sensor.red() < color_sensor.blue()) {
                    leftFront.setPower(move);
                    leftBack.setPower(move);
                    rightFront.setPower(-move);
                    rightBack.setPower(-move);
                    sleep(600);
                    stopDatMovement(leftFront, rightFront, leftBack, rightBack);
                    sleep(2000);
                    ballArm.setPosition(1.0);
                    detected = true;
                }
//bool

            }


            //more VuForia
            relicTrackables.activate();
            public enum RelicRecoveryVuMark = RelicRecoveryVuMark.from(relicTemplate); //camera

            sleep(2000);
           relicTrack.activate();
            boolean canSee = false;
 //why is this
            RelicRecoveryVuMark vuMark;

            while(!canSee) {
                vuMark = RelicRecoveryVuMark.from(Template);
                telemetry.addData("VuMark", "%s visible", vuMark);

                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    canSee = true;
                    telemetry.addData("Vumark", vuMark);
                    leftFront.setPower(0.0);
                    leftBack.setPower(0.0);
                    rightFront.setPower(0.0);
                    rightBack.setPower(0.0);

                }
                else {
                    telemetry.addData("VuMark", "not visible");
                    //add some stuff in here

                    leftFront.setPower(slowMove);
                    leftBack.setPower(slowMove);
                    rightFront.setPower(slowMove);
                    rightBack.setPower(slowMove);
                }

                telemetry.update();
//            }
             if (testy_color_sensor.red() > testy_color_sensor.blue()) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
             }
             else {
                 leftFront.setPower(-slowMove);
                 leftBack.setPower(-slowMove);
                 rightFront.setPower(-slowMove);
                 rightBack.setPower(-slowMove);
             }
        }
        leftOut.setPosition(0.0);
        rightOut.setPosition(0.0);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)");
        telemetry.update();
<<<<<<< HEAD
    }}

=======
    }
>>>>>>> 0aaa1179eab32ca6ed08583fd62c35a08fceb285
    public static void stopDatMovement(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4)
    {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
