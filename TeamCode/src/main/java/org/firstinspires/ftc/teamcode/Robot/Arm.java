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

    public Servo LR, RR;

    public Servo grab, spin;

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

    public Arm(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad, DriveTrain driveTrain, boolean New) {

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        gamepad2 = gamepad;

        myDrive = driveTrain;

        RR = hardwareMap.get(Servo.class, "RR");
        LR = hardwareMap.get(Servo.class, "LR");

        spin = hardwareMap.get(Servo.class, "spin");
        grab = hardwareMap.get(Servo.class, "grab");
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
        while(Slide.getCurrentPosition() > 25) {
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

        double constant = 0; //Kp constant TBD

        double speed = (Slide.getTargetPosition() - Slide.getCurrentPosition()) * constant;

        while(Slide.getCurrentPosition() < Slide.getTargetPosition()){
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
        Slide.setTargetPosition(50);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.9);
        while(Slide.getCurrentPosition() < 40) {

            myDrive.mecanumDrive();
        }
    }

    public void newArm()
    {
        if(gamepad2.left_stick_y != 0)
        {
            Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(Slide.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            Slide.setPower(gamepad2.left_stick_y/2);
        }

        if(gamepad2.x)
        {
            spin.setPosition(0);
        }
        if(gamepad2.y)
        {
            spin.setPosition(.8);
        }
        if(gamepad2.a)
        {
            grab.setPosition(.8);
        }
        if(gamepad2.b)
        {
            grab.setPosition(.1);
        }


        if(gamepad2.left_bumper)
        {
            LR.setPosition(.8);
            RR.setPosition(.3);
        }
        if(gamepad2.right_bumper)
        {
            LR.setPosition(.3);
            RR.setPosition(.8);
        }
        if(gamepad2.dpad_down)
        {
            grab.setPosition(.25);
        }
    }

    public void closeRepos()
    {
        LR.setPosition(.3);
        RR.setPosition(.8);
    }

    public void openRepos()
    {
        LR.setPosition(.8);
        RR.setPosition(.3);
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
            Intake.setPosition(0.1);
        }
        if(gamepad2.left_bumper)
        {
            Intake.setPosition(1);
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
