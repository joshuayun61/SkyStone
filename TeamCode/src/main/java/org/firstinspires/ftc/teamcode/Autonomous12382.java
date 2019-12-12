package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.DriveTrain;

@Autonomous
public class Autonomous12382 extends LinearOpMode {
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        waitForStart();
        if (opModeIsActive()) {
            driveTrain.forward(15, 0.5);

            //sleep(2000);

        }
    }
}
