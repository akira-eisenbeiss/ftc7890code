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

import java.util.Locale;
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

/*
7890 Space Lions 2018 "FULL AUTO FINAL"
GOALS: jewel, vuforia, glyph scoring, parking
 */
//has become the red side

@Autonomous(name="FULL AUTO FINAL", group="LinearOpMode")
public class FULL_AUTO extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    //MOTORS
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor leftIntake, rightIntake;
    DcMotor drawbridge;

    //SERVOS
    CRServo ballArm;

    //SENSORS
    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;
    ColorSensor jewelSensorL, cryptoSensor; //these two are mounted on the ballArm
    ModernRoboticsI2cRangeSensor rangeSensor;

    //COUNTERS AND BOOLEANS
    boolean sensed = false;
    /*
    USED TO SEE IF THE BALLARM SHOULD KEEP GOING
    used in[jewel, extendBallArm]
     */
    boolean turned = false;
    /*
    USED TO MAKE SURE ROBOT CONTINUOSLY TURNS
    used in [scoreTurning]
     */
    boolean stopArm = false;
    /*
    MAKES SURE JEWEL ARM CONTINUES TO EXTEND UNTIL COLOR IS SENSED
    used in [extendBallArm]
    */
    boolean scoring = false;
    /*
    USED TO MAKE THE ROBOT TURN TOWARDS CRYPTO BOX
    used in [scoreTurning]
     */
    int forwards = 1;
    /*
    USED TO MAKE SURE ROBOT MOVES TO RIGHT DISTANCE FROM WALL
    used in [dividerCount]
    */
    int cntr = 0;
    /*
    USED TO COUNT THE DIVIDERS THAT WE PASS
    used in [cryptoCheck, dividerCount]
     */
    int targetCount;
    /*
    THE DESIRED AMOUNT OF DIVIDERS DETECTED
    used in [cryptoCheck, dividerCount]
     */
    //SPEEDS
    double out = 0.3;
    /*
    THE POWER SET TO MOVE THE BALLARM OUT AND IN
    used in[jewel, extendBallArm]
     */
    int extendArm = 4750;
    /*
    THE DURATION OF THE BALLARM EXTENDING
    not currently used
     */
    private final static double move = 0.4;
    /*
    SPEED OF THE WHEEL MOTORS
    used in all movement methods [end of the code]
    */
    double sleepTime;
    long sleepTimeLong;
    //GYRO SENSOR CODES
    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;

    //ENCODERS
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    //VUFORIA
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException{

        //MOTORS
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");
        drawbridge = hardwareMap.dcMotor.get("drawbridge");

        //SENSORS
        jewelSensorL = hardwareMap.colorSensor.get("jewel sensor L");
        cryptoSensor = hardwareMap.colorSensor.get("jewel sensor R");
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) MRGyro;
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound range");

        //SERVOS
        ballArm = hardwareMap.crservo.get("ball arm");

        waitForStart();
        /*
        Below are our actual methods for
        autonomous...
        */
        //commented out for testing
        jewel(); //detects enemy team's jewel and knocks it off
 //       vuMark(); //scans the pictograph to decide which column to deposit glyph
   //     dividerCount(); //moves us to the correct column and deposits the glyph
     //   scoreTurning(); //correts our orientation in prep. for depositing glyph
    }
    //KNOCKS OFF THE CORRECT JEWEL
    public void jewel(){
        while(!sensed && opModeIsActive()) {
            extendBallArm();
            String color = isColor();
            if (color.equals("RED")){
                telemetry.addData("DETECTED COLOR", "RED");
                telemetry.update();
                //right strafe
                encoderDrive(0.3, 3, -3, -3, 3, 30);
                ballArm.setPower(-out);
                sleep(extendArm);
                sensed = true;
                break;
            }
            else if (color.equals("BLUE")) {
                telemetry.addData("DETECTED COLOR", "BLUE");
                telemetry.update();
                //left strafe
                encoderDrive(0.3, -3, 3, 3, -3, 30);
                ballArm.setPower(-out);
                sleep(extendArm);
                sensed = true;
                break;
            }
            else if (color.equals("SKIP")){
                telemetry.addData("DETECTED COLOR", "SKIP");
                telemetry.update();
                ballArm.setPower(-out);
                sleep(extendArm);
                sensed = true;
                break;
            }
            else if (!opModeIsActive()) {
                break;
            }
            else{
                telemetry.addData("DETECTED COLOR", "NONE");
                telemetry.update();
                //This shouldn't happen...
            }
        }
    }
    //EXTENDS JEWEL ARM UNTIL IT SENSES JEWEL
    public void extendBallArm(){
        while (!stopArm && opModeIsActive()) {
            if (jewelSensorL.blue() > jewelSensorL.red() || jewelSensorL.red() > jewelSensorL.blue()) {
                ballArm.setPower(0);
                stopArm = true;
                break;
            }
            else if (!opModeIsActive()) {
               break;
            }
            else {
                ballArm.setPower(out);
                telemetry.addLine("extending ball arm");
                telemetry.update();
            }
        }
    }
    //DETECTS COLOR
    String colorDetected;
    public String isColor(){
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
    //POSITIONS ROBOT AT CIPHER
    public void dividerCount(){
        telemetry.addLine("dividerCount ACTIVATED");
        telemetry.update();
        //moving off balancing plate
        encoderDrive(0.5,-5,5,5,-5,30); //arbitrary values
        //moving forward so that we can sense cryptobox
        encoderDrive(0.5,1,1,1,1,30); //arbitrary values
        /*
        WHILE THE AMOUNT OF DIVIDERS SENSED IS
        LESS THAN THE AMOUNT OF DIVIDERS DESIRED,
        WE KEEP STRAFING... this breaks when we
        have passed the correct number of dividers.
        */
        targetCount = 2;
        while (targetCount > cntr && opModeIsActive()) {
            rightStrafe(leftFront, leftBack, rightFront, rightBack);
            telemetry.addLine("strafing to cypher");
            telemetry.update();
            if (cryptoSensor.blue() > cryptoSensor.red()) {
                cntr++;
            }
            /*
            SLEEP SO WE DON'T DETECT
            THE SAME CRYPTOBOX TWICE
            */
            sleep(150); //TODO: fix this value!
        }
        if (targetCount == cntr) {
            //BACKING UP TO AVOID HITTING THE CRYPTOBOX
            encoderDrive(0.5,1,1,1,1,30); //arbitrary values
            telemetry.addLine("READY TO SCORE");
            telemetry.update();
        }
    }
    //TURNS ROBOT AT CIPHER
    public void scoreTurning() {
        while (!turned && opModeIsActive()) {
            int heading = MRGyro.getHeading();
            rotateCW(leftFront,leftBack,rightFront,rightBack);
            telemetry.addLine("TURNING...");
            telemetry.update();

            if (heading > targetHeadingGlyph - 10 && heading < targetHeadingGlyph + 10) {
                stopDatMovement(leftFront, leftBack, rightFront, rightBack);
                turned = true;
                telemetry.addLine("TURNING COMPLETE");
                telemetry.update();
                //moving to get closer to cryptobox
                encoderDrive(0.5,1,1,1,1,30); //arbitrary values
                //to score the glyph
                scoreGlyph();
            }
        }
    }
    //SCORES GLYPH
    public void scoreGlyph(){
        while(!scoring && opModeIsActive()) {
            telemetry.addLine("RELEASING GLYPH...");
            telemetry.update();
            leftIntake.setPower(0.7);
            rightIntake.setPower(-0.7);
        }
    }
    //VUFORIA METHOD
    public void vuMark(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AYvh06j/////AAAAmbygSYf3jk52rJ+QebMiuY+KBW8AuYZAvwG7yH+LzSrVKE/bD6Xhi3ksUHXEagcXs9z59YKe9ihnuvgO9Wk5gl/E8sGjKKiY9Por4JQZk7aTe0G0FcL6SLIjn5PHRdJNkFp8oLyPhhK2W/b2zM4+OqMRJjex2Q8jFOJNqFGr1HroVBnhNU+6JXZhBNhP+gw6KPwui7TEVgGiDSz2FiYMzER9D27szSZsTx2LWy6XjAJjSbKQ1WnXCs5EQh2uEE0hFjqw3DrVyXjG731O1RzGPUMkKgviF6JyZS8k3VM20NSBf+FBCJbXe5gfHE5wqfC+G51ZUQTzUbaXSmdC2vcuOQ3hVN+Irbh8tZG2hWbaOKu8";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
        boolean detectedPicto = false;
        RelicRecoveryVuMark vuMark;
        while(detectedPicto == false && opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            while(vuMark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive( )) {
                runtime.reset();
                rotateCCWSlow(leftFront, leftBack, rightFront, rightBack);
                sleepTime = runtime.seconds();
                sleepTimeLong = (new Double(sleepTime)).longValue();
            }
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                detectedPicto  = true;
                stopDatMovement(leftFront,leftBack,rightFront,rightBack);
                rotateCW(leftFront, leftBack, rightFront, rightBack);
                sleep(sleepTimeLong);
                telemetry.addData("Vuforia", vuMark);
                sleep(1000);
                //CHANGING TARGET COUNT
                if (vuMark == RelicRecoveryVuMark.LEFT)
                    targetCount = 1;
                else if(vuMark == RelicRecoveryVuMark.CENTER)
                    targetCount = 2;
                else if(vuMark == RelicRecoveryVuMark.RIGHT)
                    targetCount = 3;
            }
            else {
                telemetry.addData("Vuforia", "NOT DETECTED");
                leftStrafe(leftFront,leftBack,rightFront,rightBack);
                //TODO: FIX THE STRAFING DIRECTIONS!?
            }
            telemetry.update();
        }
    }
    //Setting up encoders
    public void encoderDrive(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches, double timeOut) {

        //sets up target for wheels
        int leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget;

        if (opModeIsActive()) {
            //the reversing of left side may cause some problems, make sure ot comment them out
            //the inch values get converted into actual inches here
            leftFrontTarget = (leftFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH)) * -1;
            leftBackTarget = (leftBack.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH)) * -1;
            rightFrontTarget = rightFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            rightBackTarget = rightBack.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

            //setting target
            leftFront.setTargetPosition(leftFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            rightBack.setTargetPosition(rightBackTarget);

            //turning on RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //reset timer, set motors to absolute value of speed(shown in method parameters)
            runtime.reset();

            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeOut) &&
                    (leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy())) {

                telemetry.addData("movement", "Running to %7d :%7d : %7d :%7d",
                        leftFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        rightBack.getCurrentPosition()
                );
                telemetry.addData("targets", "Running to %7d :%7d : %7d :%7d ",
                        leftFrontTarget,
                        leftBackTarget,
                        rightFrontTarget,
                        rightBackTarget);
                telemetry.update();
            }

            // Stop dat movement
            stopDatMovement(leftFront, leftBack, rightFront, rightBack);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //---------------------------//

    //methods for basic movements
    public static void stopDatMovement(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    //strafe methods
    public void leftStrafe(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(-move);
        motor3.setPower(move);
        motor4.setPower(-move);
    }
    public void rightStrafe(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-move);
        motor2.setPower(move);
        motor3.setPower(-move);
        motor4.setPower(move);
    }
    public void rightStrafeSlow(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-0.2);
        motor2.setPower(0.2);
        motor3.setPower(-0.2);
        motor4.setPower(0.2);
    }
    //y-axis movement methods
    public void moveForward(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-move);
        motor2.setPower(-move);
        motor3.setPower(move);
        motor4.setPower(move);
    }
    public void moveForwardSlow(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-0.2);
        motor2.setPower(-0.2);
        motor3.setPower(0.2);
        motor4.setPower(0.2);
    }
    public void moveBackwards(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(move);
        motor3.setPower(-move);
        motor4.setPower(-move);
    }
    //rotating methods
    public static void rotateCW(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-move);
        motor2.setPower(-move);
        motor3.setPower(-move);
        motor4.setPower(-move);
    }
    public static void rotateCCW(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(move);
        motor2.setPower(move);
        motor3.setPower(move);
        motor4.setPower(move);
    }
    public static void rotateCCWSlow(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(0.1);
        motor2.setPower(0.1);
        motor3.setPower(0.1);
        motor4.setPower(0.1);
    }
    public static void rotateCWSlow(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        motor1.setPower(-0.1);
        motor2.setPower(-0.1);
        motor3.setPower(-0.1);
        motor4.setPower(-0.1);
    }
}
