package org.firstinspires.ftc.teamcode.Regionals_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Arm;

@TeleOp(name = "Nebula TeleOp")
public class NewTeleOp12382 extends LinearOpMode {

    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1, true);
        Arm arm  = new Arm(telemetry, hardwareMap, gamepad2, driveTrain, true);
        waitForStart();
        while (opModeIsActive())
        {
            driveTrain.mecanumDrive();
            arm.newArm();
            driveTrain.suck();
            telemetry.addData("Slide", arm.Slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
