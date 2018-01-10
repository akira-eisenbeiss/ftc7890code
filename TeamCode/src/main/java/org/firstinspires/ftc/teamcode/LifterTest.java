package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="complete tele op2", group="Tele Op")
public class LifterTest extends OpMode {

    DcMotor lift;
    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("liftMotor");
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop(){
        lift.setPower(gamepad1.left_stick_y);
    }
}
