package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="complete tele op", group="Tele Op")
public class TeleOpFull extends OpMode {

    //directions
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.FORWARD;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.REVERSE;

    // declared the opmode members
    private ElapsedTime runtime = new ElapsedTime();
    //drive
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    //lift and intake
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor liftMotor = null;

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //wheels
        leftFront = hardwareMap.get(DcMotor.class, "left front");
        leftBack = hardwareMap.get(DcMotor.class, "left back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        rightBack = hardwareMap.get(DcMotor.class, "right back");
        //mechanism #1
        leftIntake = hardwareMap.get(DcMotor.class, "left intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right intake");
        liftMotor = hardwareMap.get(DcMotor.class, "lift motor");
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
    public void loop(){

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double liftRaise;
            double liftLower;


            // POV drivings controls
            float drive = -gamepad1.left_stick_y;
            float turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive - turn, -1.0, 1.0);
            rightPower = Range.clip(drive + turn, -1.0, 1.0);
        
        /*
            //tank mode version
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;
        */
            //wheel lift mechanism controls
            float leftTrigger1 = gamepad1.left_trigger;
            float rightTrigger1 = gamepad1.right_trigger;
            boolean gamepad1A = gamepad1.a;
            boolean gamepad1B = gamepad1.b;
            liftRaise = Range.clip(leftTrigger1, -0.75, 0.75);
            liftLower = Range.clip(rightTrigger1, 0.75, -0.75);

            // Send calculated power to wheels
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            //the ifs that control the lift mechanism
            if (leftTrigger1 > 0) {
                liftMotor.setPower(liftRaise);
            } else {
                liftMotor.setPower(leftTrigger1);
            }
            if (rightTrigger1 > 0) {
                liftMotor.setPower(liftLower);
            } else {
                liftMotor.setPower(rightTrigger1);
            }

            int x = 0;
            //the ifs that are used for toggling the intake mechanism
            if (gamepad1A) {
                x++;
                if (x % 2 != 0) {
                    leftIntake.setPower(1.0);
                    rightIntake.setPower(1.0);
                } else if (x % 2 == 0){
                    leftIntake.setPower(0.0);
                    rightIntake.setPower(0.0);
                }
            }
            int y = 0;
            if (gamepad1B) {
                y++;
                if (y % 2 != 0) {
                    leftIntake.setPower(-1.0);
                    rightIntake.setPower(-1.0);
                } else if (y % 2 == 0){
                    leftIntake.setPower(0.0);
                    rightIntake.setPower(0.0);
                }
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
    }
}
