package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Clean")
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot12382 robot = new Robot12382(telemetry, hardwareMap);

        waitForStart();

        Thread driveTrainAutonomous = new Thread(robot.driveTrainAutonomous);

        while (opModeIsActive()) {

        }
    }
}
