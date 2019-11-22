package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends LinearOpMode {
    public void runOpMode() {

        //Instantiate local class instances
        hardware hardware = new hardware(telemetry, hardwareMap);
        hardware.setupMotors();
        //hardware.setupServos();

        /*DcMotor FL = hardware.FL;
        DcMotor FR = hardware.FR;
        DcMotor BL = hardware.BL;
        DcMotor BR = hardware.BR;*/
        DcMotor Slide = hardware.Slide;
        Servo intake = hardware.intake;
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int position = 0;

        waitForStart();

        while(opModeIsActive()) {
            /*FL.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
            FR.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
            BL.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
            BR.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);*/

            if (gamepad1.dpad_up) {
                position += 3;
                Slide.setTargetPosition(position);
                Slide.setPower(0.5);
            }

            if (gamepad1.dpad_down) {
                position -= 3;
                Slide.setTargetPosition(position);
                Slide.setPower(0.5);
            }

            if (gamepad1.x) {
                intake.setPosition(0.5);
            }
            if (gamepad1.y) {
                intake.setPosition(1);
            }
        }
    }
}
