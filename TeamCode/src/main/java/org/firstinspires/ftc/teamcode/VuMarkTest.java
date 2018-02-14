package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="vumark detection test", group="LinearOpMode")
public class VuMarkTest extends LinearOpMode {

    //vuforia
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AYvh06j/////AAAAmbygSYf3jk52rJ+QebMiuY+KBW8AuYZAvwG7yH+LzSrVKE/bD6Xhi3ksUHXEagcXs9z59YKe9ihnuvgO9Wk5gl/E8sGjKKiY9Por4JQZk7aTe0G0FcL6SLIjn5PHRdJNkFp8oLyPhhK2W/b2zM4+OqMRJjex2Q8jFOJNqFGr1HroVBnhNU+6JXZhBNhP+gw6KPwui7TEVgGiDSz2FiYMzER9D27szSZsTx2LWy6XjAJjSbKQ1WnXCs5EQh2uEE0hFjqw3DrVyXjG731O1RzGPUMkKgviF6JyZS8k3VM20NSBf+FBCJbXe5gfHE5wqfC+G51ZUQTzUbaXSmdC2vcuOQ3hVN+Irbh8tZG2hWbaOKu8";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark;

        while (opModeIsActive()) {
            vumark();
        }
    }

    public void vumark() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while(vuMark == RelicRecoveryVuMark.UNKNOWN){
            telemetry.addData("VuMark", "UNKNOWN");
            telemetry.update();
        }
        while(vuMark == RelicRecoveryVuMark.LEFT){
            telemetry.addData("VuMark", "LEFT");
            telemetry.update();
        }
        while(vuMark == RelicRecoveryVuMark.CENTER){
            telemetry.addData("VuMark", "CENTER");
            telemetry.update();
        }
        while(vuMark == RelicRecoveryVuMark.RIGHT){
            telemetry.addData("VuMark", "RIGHT");
            telemetry.update();
        }
    }
}
