package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot12382 robot = new Robot12382(telemetry, hardwareMap);

        robot.openCVTestBench.setup();

        waitForStart();

        robot.openCVTestBench.start();

        while (opModeIsActive()) {
            telemetry.addData("Color is Red: ", robot.openCVTestBench.getColor());
            telemetry.update();
        }

        robot.openCVTestBench.stop();
    }
}
