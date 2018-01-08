package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="lift test", group="Tele Op")
public class LiftTest extends OpMode {
    //sorry about these strings, btw
    public final static String LIFTMOTOR = "liftMotor";
    private DcMotor liftMotor;

    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotor.class, LIFTMOTOR);
    }
    @Override
    public void loop(){
        float leftTrigger = gamepad1.left_trigger;
        float rightTrigger = gamepad1.right_trigger;

        if (leftTrigger < 0)
            liftMotor.setPower(1.0);
        if (rightTrigger < 0)
            liftMotor.setPower(0.0);

    }
}
