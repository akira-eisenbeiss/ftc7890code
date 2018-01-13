package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TurnTest", group="Linear Op")
public class TurnTest extends LinearOpMode {
    
    DcMotor leftFront, leftBack, rightFront, rightBack;
    
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("leftFront");
        
        waitForStart();
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
        
        while (opModeIsActive()) {
            leftFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightFront.setPower(0.5);
            rightBack.setPower(0.5);
        }
    }
}
