package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.DriveTrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Clean")
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot12382 robot = new Robot12382(telemetry, hardwareMap);

        waitForStart();

        robot.driveTrain.strafe(DriveTrain.strafeDirection.FORWARD, 20, 0.5);

        while (opModeIsActive()) {

        }
    }
}
