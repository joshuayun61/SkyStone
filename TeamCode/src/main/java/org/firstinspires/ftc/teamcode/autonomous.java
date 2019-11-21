package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Autonomous", group = "Linear OpMode")

public class autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Classes that have majority of the methods.
        TensorMethods vision  = new TensorMethods(telemetry, hardwareMap);
        autonomousCommands commands = new autonomousCommands(telemetry, hardwareMap);

        vision.setup();
        //autonomousCommands commands = new autonomousCommands();
        while(!opModeIsActive()) {
            vision.runTfod();
        }
        if (opModeIsActive()) {

            vision.stopTfod();

            while (opModeIsActive()) {}
        }
    }


}

