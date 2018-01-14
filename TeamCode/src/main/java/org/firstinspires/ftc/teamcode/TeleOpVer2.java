package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="complete tele op2", group="Tele Op")
public class TeleOpVer2 extends OpMode {
    //sorry about these strings, btw
    public final static String LEFTFRONT = "leftFront";
    public final static String LEFTBACK = "leftBack";
    public final static String RIGHTFRONT = "rightFront";
    public final static String RIGHTBACK = "rightBack";
    //names of the intake wheels
    public final static String LEFTINTAKE = "leftIntake";
    public final static String RIGHTINTAKE = "rightIntake";
    //name of the motor that does the lift
    public final static String LIFTMOTOR = "liftMotor";
    //servos
    //  public final static String CLAMPSERVO = "clampServo";
    // public final static String VERTSERVO = "vertServo";
    public final static String BALLARM = "ballArm";
    //scissor
    // public final static String SCISSOR = "scissor";
    //directions
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;
    // declared the opmode members
    private ElapsedTime runtime = new ElapsedTime();
    //drive
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    //lift and intake
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor liftMotor;
    //servos
    // private Servo clampServo;
    //   private Servo vertServo;
    private Servo ballArm;
    //scissor
    private DcMotor scissor;

    //intake outtake variables
    int intakePower = 0;

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //wheels
        leftFront = hardwareMap.get(DcMotor.class, LEFTFRONT);
        leftBack = hardwareMap.get(DcMotor.class, LEFTBACK);
        rightFront = hardwareMap.get(DcMotor.class, RIGHTFRONT);
        rightBack = hardwareMap.get(DcMotor.class, RIGHTBACK);
        //mechanism #1
        leftIntake = hardwareMap.get(DcMotor.class, LEFTINTAKE);
        rightIntake = hardwareMap.get(DcMotor.class, RIGHTINTAKE);
        liftMotor = hardwareMap.get(DcMotor.class, LIFTMOTOR);
        //servos
        ballArm = hardwareMap.get(Servo.class, BALLARM);

        /* clampServo = hardwareMap.get(Servo.class, CLAMPSERVO);
        //vertServo = hardwareMap.get(Servo.class, VERTSERVO);
        //scissor
        scissor = hardwareMap.get(DcMotor.class, SCISSOR);*/
//hiiii
        //motor directions
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);
        //intake and lift
        leftIntake.setDirection(LEFTDIRECTION);
        rightIntake.setDirection(RIGHTDIRECTION);
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop(){

        // POV drivings controls
        float drive = -gamepad1.left_stick_y;
        float turn = gamepad1.right_stick_x;
        float strafe = -gamepad1.left_stick_x;

        //wheels
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        //wheel lift and intake mechanism controls
      //  float leftTrigger2 = gamepad2.left_trigger;
       // float rightTrigger2 = gamepad2.right_trigger;
        //intake wheels
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
     //   double liftRaise = Range.clip(-leftTrigger2, -1.0, 0.0);
      //  double liftLower = Range.clip(rightTrigger2, 0.0, 1.0);
        //relic mech
      //  float leftTrigger1 = gamepad1.left_trigger;
      //  float rightTrigger1 = gamepad1.right_trigger;
       // double scissorOut = Range.clip(leftTrigger1, -1.0, 0.0);
       // double scissorIn = Range.clip(rightTrigger1, 0.0, 1.0);
        //clamp
       // float leftStick2 = -gamepad2.left_stick_y;
        float rightStick2 = gamepad2.right_stick_y;

        //just the servo arm up so it doesn't break
        ballArm.setPosition(1.0);

        //wheel
        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);

        float leftsticky = gamepad2.left_stick_y;
        liftMotor.setPower(leftsticky);

        /*
        //lift mechanism
        if (leftTrigger2 > 0) {
            liftMotor.setPower(liftRaise);
        } else {
            liftMotor.setPower(0.0);
        }
        if (rightTrigger2 > 0) {
            liftMotor.setPower(liftLower);
        } else {
            liftMotor.setPower(0.0);
        }
        */

        // hoot897 @arden

        if(gamepad2A && intakePower == 0) // in
        {
            leftIntake.setPower(1.0);
            rightIntake.setPower(1.0);
            intakePower ^= 1;
        }
        else if(gamepad2A && intakePower == 1)// out
        {
            leftIntake.setPower(-1.0);
            rightIntake.setPower(-1.0);
            intakePower ^= 1;
        }
        else if(gamepad2B)
        {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
//dab skrahh - erin

/*
        //servo clamp that grabs the relic
        if (leftStick2 != 0.0) {
            clampServo.setPosition(leftStick2);
        }
        //servo that changes the position of the grabbed relic
        if (rightStick2 != 0.0) {
        vertServo.setPosition(rightStick2);
        }
        //scissor
        if (leftTrigger1 > 0) {
            liftMotor.setPower(scissorOut);
        } else {
            liftMotor.setPower(leftTrigger1);
        }
        if (rightTrigger1 > 0) {
            liftMotor.setPower(scissorIn);
        } else {
            liftMotor.setPower(rightTrigger1);
        }
*/

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
