package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp12382 with Abstraction")

public class TeleOp12382 extends LinearOpMode
{

    @Override
    public void runOpMode() {

        TeleOpCommands commands = new TeleOpCommands(telemetry, hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while(opModeIsActive()) {
            commands.mecanum();
            commands.turn180();
            commands.slideMotor();
        }
    }
}