/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class TeleOp7890 extends LinearOpMode {

    //names of wheel motors
    public final static String LEFTFRONT = "leftFront";
    public final static String LEFTBACK = "leftBack";
    public final static String RIGHTFRONT = "rightFront";
    public final static String RIGHTBACK = "rightBack";
    //names of the intake wheels
    public final static String LEFTINTAKE = "leftIntake";
    public final static String RIGHTINTAKE = "rightIntake";
    //name of the motor that does the lift
    public final static String LIFTMOTOR = "liftMotor";

    //directions
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.FORWARD;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.REVERSE;

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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
            //wheels
        leftFront  = hardwareMap.get(DcMotor.class, LEFTFRONT);
        leftBack = hardwareMap.get(DcMotor.class, LEFTBACK); //left = forward
        rightFront = hardwareMap.get(DcMotor.class, RIGHTFRONT);
        rightBack = hardwareMap.get(DcMotor.class, RIGHTBACK); //right = backwards
            //mechanism #1
        leftIntake = hardwareMap.get(DcMotor.class, LEFTINTAKE);
        rightIntake = hardwareMap.get(DcMotor.class, RIGHTINTAKE);
        liftMotor = hardwareMap.get(DcMotor.class, LIFTMOTOR);
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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
//hi its arden
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double liftRaise;
            double liftLower;

            // POV drivings controls
            float drive = -gamepad1.left_stick_y;
            float turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, 1.0, -1.0) ;
            rightPower   = Range.clip(drive - turn, 1.0, -1.0) ;

            //wheel lift mechanism controls
            float leftTrigger1 = gamepad1.left_trigger;
            float rightTrigger1 = gamepad1.right_trigger;
            boolean toggleIn = gamepad1.a;
            boolean toggleOut = gamepad1.b;
            liftRaise = Range.clip(leftTrigger1, -1.0, 1.0);
            liftLower = Range.clip(rightTrigger1, 1.0, -1.0);

            // Send calculated power to wheels
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            //the ifs that control the lift mechanism
            if(leftTrigger1 > 0){
                liftMotor.setPower(liftRaise);
            }else{

            }
            if(rightTrigger1 > 0)
            {
                liftMotor.setPower(liftLower);
            }else{

            }

            int x = 0;
            //the ifs that are used for toggling the intake mechanism
            if(toggleIn){
                x++;
                if(x%2!=0){
                    leftIntake.setPower(1.0);
                    rightIntake.setPower(-1.0);
                }
            }
            int y = 0;
            if(toggleOut){
                y++;
                if(y%2!=0){
                    leftIntake.setPower(-1.0);
                    rightIntake.setPower(1.0);
                }
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
