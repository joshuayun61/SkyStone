package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto", group = "Linear OpMode")

public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Classes that have majority of the methods.
        TensorMethods vision  = new TensorMethods(telemetry, hardwareMap);
        AutoCommands commands = new AutoCommands(telemetry, hardwareMap);
        VuforiaNavigation vuforia = new VuforiaNavigation(telemetry, hardwareMap);

        vuforia.addFieldObjects();
        vuforia.setFieldObjectLocations();
        vuforia.vuforiaSetup();
        //AutoCommands commands = new AutoCommands();
        while(!opModeIsActive()) {
        }
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                vuforia.vuforiaRun();
            }
            vuforia.vuforiaStop();
        }
    }
}

