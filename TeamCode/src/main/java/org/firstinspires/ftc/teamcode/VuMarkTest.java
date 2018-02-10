package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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


//@Autonomous(name="vumark detection test", group="LinearOpMode")
public class VuMarkTest extends LinearOpMode {

    //vuforia
    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    VuforiaTrackable relicTemplate = relicTrackables.get(0);

    @Override
    public void runOpMode() {
        parameters.vuforiaLicenseKey = "AcoS+YP/////AAAAGTq922ywuU6FquBqcm2CeatGNf2voKamgXI1KwF7yLiQKP+RqBNrI4ND0i98TsuYnBytFG0YYUz2+4wvHBN5pz+/CacheTAG6upbc95Ts0UJgGRg0aTLaVzdYUQUI5dRlAh50DsGYdPkabTZmPO+5EYj79XDDHhok7wTZDb6ZyiCLlzXtM5EZ9nyiWQxz6XJ3M7Q+m4nVuaAdvWN+qwkQsqohSoxB8TNI4dDYlSMQbbO6d3SkCgfXy4K8y/lBNDF8suTeSgNY0YGs/N5FIYTLa+eyu+r3kbf2ig0EsL1Er+AhLZkVDpksvMp+MMBdDVyi6JDjr4E+P2D82ztt8Ex0aoR+h0n4RyRnkS+G4FB4wRD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();
        relicTrackables.activate();

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
