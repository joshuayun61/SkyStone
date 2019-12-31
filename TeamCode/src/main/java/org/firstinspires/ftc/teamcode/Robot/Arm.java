package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends LinearOpMode {

    DcMotor Slide;

    public Servo Intake;

    private DriveTrain myDrive;

    //field constants
    int buildPlateHeight = 700;
    int blockHeight = 1000;

    public Arm(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad, DriveTrain driveTrain) {

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        gamepad2 = gamepad;

        myDrive = driveTrain;

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

    public void ground()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(5);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.getCurrentPosition() > 20) {
            telemetry.addData("Slide Tick",slidePosition());
            telemetry.update();
            myDrive.mecanumDrive();
        }
    }
    public void raisePH()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(buildPlateHeight);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(slidePosition() > 710 || slidePosition() < 690) {
            telemetry.addData("Slide Tick",slidePosition());
            telemetry.update();
            myDrive.mecanumDrive();
        }
    }
    public void raisePH(int extra)
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(buildPlateHeight + extra);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.getCurrentPosition() < buildPlateHeight + 550) {
            telemetry.addData("Slide Tick",slidePosition());
            telemetry.update();
            myDrive.mecanumDrive();
        }
    }
    public void raiseBH()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(Slide.getCurrentPosition() + blockHeight);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.isBusy()) {

            myDrive.mecanumDrive();
        }
    }
    public void lowerBH()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(Slide.getCurrentPosition() - blockHeight);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.isBusy()) {}
    }
    public void blockLift()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(20);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.isBusy()) {

            myDrive.mecanumDrive();
        }
    }


    public void moveArm() {

        if(gamepad2.right_trigger > 0)
        {
            myDrive.slowStrafeRight();
        }
        if(gamepad2.left_trigger > 0)
        {
            myDrive.slowStrafeleft();
        }

        if(gamepad2.left_stick_y != 0)
        {
            Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(Slide.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            Slide.setPower(-gamepad2.left_stick_y/1.5);
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
        if(gamepad2.x)
        {
            ground();
        }
        if(gamepad2.b)
        {
            blockLift();
        }
        if(Slide.getCurrentPosition() > 4100)
        {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setTargetPosition(4000);
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
        Intake.setPosition(.18);
    }
}
