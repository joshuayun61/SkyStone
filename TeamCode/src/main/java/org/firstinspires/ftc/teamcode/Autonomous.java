package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Linear OpMode")

public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Classes that have majority of the methods.
        TensorMethods vision  = new TensorMethods(telemetry, hardwareMap);
        AutoCommands commands = new AutoCommands(telemetry, hardwareMap);

        vision.setup();
        //AutoCommands commands = new AutoCommands();
        while(!opModeIsActive()) {
            vision.runTfod();
        }
        if (opModeIsActive()) {

            vision.stopTfod();

            while (opModeIsActive()) {}
        }
    }
}

