package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "sensor_ods".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "ods test", group= "Sensor")
public class ODSTest extends LinearOpMode {

    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    double distance;

    @Override
    public void runOpMode() {

        // get a reference to our Light Sensor object.
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

          //  distance = odsSensor.getRawLightDetected();

            // send the info back to driver station using telemetry function.
           // telemetry.addData("Raw",    distance);
            telemetry.addData("Normal", odsSensor.getLightDetected());

            telemetry.update();
        }
    }
}
