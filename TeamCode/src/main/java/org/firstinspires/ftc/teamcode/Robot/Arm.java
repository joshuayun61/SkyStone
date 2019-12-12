package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends LinearOpMode {

    DcMotor Slide;

    Servo Intake;

    public Arm(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad) {

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        gamepad2 = gamepad;
    }

    public void runOpMode() {}

    public void moveArm() {

        if(gamepad2.left_stick_button)
        {
            Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(gamepad2.a)
        {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setTargetPosition(5);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(.9);
            while(Slide.isBusy()) {}
        }
        if(gamepad2.b && !gamepad1.start && !gamepad2.start)
        {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setTargetPosition(1600);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
            while(Slide.isBusy()) {}
        }
        if(gamepad2.y)
        {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setTargetPosition(2750);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
            while(Slide.isBusy()) {}
        }
        if(Slide.getCurrentPosition() > 5000)
        {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setTargetPosition(2000);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(.8);
            while(Slide.isBusy())
            {

            }
        }

        if(gamepad2.right_bumper)
        {
            Intake.setPosition(.4);
        }
        if(gamepad2.left_bumper)
        {
            Intake.setPosition(.9);
        }

    }
}
