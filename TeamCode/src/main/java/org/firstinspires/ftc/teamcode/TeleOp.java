package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot12382 robot = new Robot12382(telemetry, hardwareMap);

        waitForStart();

        Thread driveTrainThread = new Thread(robot.driveTrainTeleOp);

        while (opModeIsActive()) {
            robot.driveTrainTeleOp.mecanumDrive(gamepad1);
            telemetry.update();
        }

    }
}
