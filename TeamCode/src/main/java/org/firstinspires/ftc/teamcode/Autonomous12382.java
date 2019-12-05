package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous", group = "Linear OpMode")

public class Autonomous12382 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Classes that have majority of the methods.
        //TensorMethods vision  = new TensorMethods(telemetry, hardwareMap);
        AutoCommands commands = new AutoCommands(telemetry, hardwareMap);
        VuforiaNavigation vuforia = new VuforiaNavigation(hardwareMap, telemetry);

        vuforia.setupVuforia();

        while(!opModeIsActive()) {

        }
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                commands.forward(0.5, 18);
                vuforia.searchVuforia();
                if (vuforia.targetVisible) {
                    commands.strafe(0.2, (int) vuforia.locateVuforia().get(0));
                    commands.openServo();
                    commands.forward(0.5, (int) vuforia.locateVuforia().get(1));
                    commands.closeServo();
                }
                else {
                    telemetry.addLine("Could not find Target, defaulting to Backup Plan");
                    //Add backup plan here
                }
            }
            vuforia.stopVuforia();
        }
    }
}

