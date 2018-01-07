package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="test", group="Tele Op")
public class java extends OpMode {

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
    public final static String LEFTCLAMP = "leftClamp";
    public final static String RIGHTCLAMP = "rightClamp";

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
    //private Servo leftClamp;
    //private Servo rightClamp;

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
        //leftClamp = hardwareMap.get(Servo.class, LEFTCLAMP);
        //  rightClamp = hardwareMap.get(Servo.class, RIGHTCLAMP);

//hiiii
        //motor directions
        //wheels
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);
        //intake and lift
        leftIntake.setDirection(LEFTDIRECTION);
        rightIntake.setDirection(RIGHTDIRECTION);
        liftMotor.setDirection(LEFTDIRECTION);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double liftRaise;
        double liftLower;
        double backStrafe;
        double frontStrafe;

        if (gamepad1.dpad_up) {
            leftFront.setPower(1);
            leftBack.setPower(1);
            rightFront.setPower(1);
            rightBack.setPower(1);
        }
        else if (gamepad1.dpad_down){
            leftFront.setPower(-1);
            leftBack.setPower(-1);
            rightFront.setPower(-1);
            rightBack.setPower(-1);
        }
        else if (gamepad1.dpad_right){
            leftFront.setPower(1);
            leftBack.setPower(-1);
            rightFront.setPower(-1);
            rightBack.setPower(1);
        }
        else if (gamepad1.dpad_left){
            leftFront.setPower(-1);
            leftBack.setPower(1);
            rightFront.setPower(1);
            rightBack.setPower(-1);
        }
        else{
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }
}

