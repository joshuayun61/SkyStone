package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.util.FastMath;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Clean")

/**
 * Note: if you want to change the direction of the movement in the Autonomous
 * you have to set new parameters with robot.driveTrain.setMoveParammeters()
 * and call Thread.run() to run it with the new parameters.
 */
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot12382 robot = new Robot12382(telemetry, hardwareMap);

        Thread drive = new Thread(robot.driveTrain);

        waitForStart();

        robot.driveTrain.setMoveParameters(DriveTrain.strafeDirection.FORWARD, 20, 0.5);

        drive.run();

        while (opModeIsActive()) {

        }
    }
}
