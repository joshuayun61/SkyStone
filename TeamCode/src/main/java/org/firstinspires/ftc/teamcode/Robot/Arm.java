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

    //field constants
    int bulidPlateHeight = 700;
    int blockHeight = 1000;

    public Arm(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad) {

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        gamepad2 = gamepad;

        Intake = hardwareMap.get(Servo.class, "Intake");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runOpMode() {}

    public int slidePosition()
    {
        return Slide.getCurrentPosition();
    }
    public void raisePH()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(bulidPlateHeight);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.isBusy()) {}
    }
    public void raiseBH()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(Slide.getCurrentPosition() + blockHeight);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.isBusy()) {}
    }
    public void lowerBH()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(Slide.getCurrentPosition() - blockHeight);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.isBusy()) {}
    }


    public void moveArm() {

        if(gamepad2.left_stick_button)
        {
            Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(Slide.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            Slide.setPower(gamepad2.left_stick_y);
        }
        if(gamepad2.a)
        {
            raisePH();
        }
        if(gamepad2.dpad_up)
        {
            raiseBH();
        }
        if(gamepad2.dpad_down)
        {
            lowerBH();
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
            Intake.setPosition(.2);
        }
        if(gamepad2.left_bumper)
        {
            Intake.setPosition(.9);
        }

    }
    public void openArm()
    {
        Intake.setPosition(.9);
    }
    public void closeArm()
    {
        Intake.setPosition(.2);
    }
}
