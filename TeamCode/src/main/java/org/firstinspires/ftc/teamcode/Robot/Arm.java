package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends LinearOpMode {

    public DcMotor Slide;

    public Servo Intake;

    public Servo LR, RR;

    public Servo grab, spin, auto_arm, auto_grab;

    public Servo tipIn;

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

        tipIn = hardwareMap.get(Servo.class, "tip");

        auto_arm = hardwareMap.get(Servo.class, "auto_arm");
        auto_grab = hardwareMap.get(Servo.class, "auto_grab");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setDirection(DcMotor.Direction.REVERSE);
    }


    public void runOpMode() {}

    public int slidePosition()
    {
        return Slide.getCurrentPosition();
    }

    public void home()
    {
        if(Slide.getCurrentPosition() > 100) {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setTargetPosition(0);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
            spin.setPosition(0);
            grab.setPosition(.1);
            while (Slide.getCurrentPosition() < 3) {
                grab.setPosition(.1);
                myDrive.mecanumDrive();
                myDrive.suck();
            }
        }

    }
    public void max()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(9000);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.95);
        while(Slide.getCurrentPosition() < 1)
        {
            myDrive.mecanumDrive();
        }
    }

    public void readyToPlace()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(3500);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(1);
        while(Slide.getCurrentPosition() < 3500)
        {
            myDrive.mecanumDrive();
        }

        spin.setPosition(.4);
    }

    public void newArm()
    {
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(gamepad2.left_stick_y != 0)
        {
            Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(Slide.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            Slide.setPower(-gamepad2.left_stick_y);
        }

        if(Slide.getCurrentPosition() >= 9200)
        {
            max();
        }

        //spin out
        if(gamepad2.x)
        {
            spin.setPosition(.78);
        }
        //spin in
        if(gamepad2.y)
        {
            spin.setPosition(0);
        }
        //open grabber
        if(gamepad2.a)
        {
            grab.setPosition(.5);

        }
        //close grabber
        if(gamepad2.b)
        {
            grab.setPosition(0);
            tipIn.setPosition(.4);

        }

        if(gamepad2.left_bumper)
        {
            closeRepos();
        }

        if(gamepad2.right_bumper)
        {
           openRepos();
        }

        if(gamepad2.dpad_up)
        {
            readyToPlace();

        }

        if(gamepad2.dpad_down)
        {
            home();
        }

        if(gamepad2.dpad_left) //rotate away
        {
            tipIn.setPosition(.4);
        }
        if(gamepad2.dpad_right) //rotate in
        {
            tipIn.setPosition(.7);
        }



        if(gamepad2.right_stick_x > 0)
        {
            spin.setPosition(spin.getPosition() + .005);
        }
        if(gamepad2.right_stick_x < 0)
        {
            spin.setPosition(spin.getPosition() - .005);
        }
        if(gamepad2.right_stick_y > 0)
        {
            grab.setPosition(grab.getPosition() + .005);
        }
        if(gamepad2.right_stick_y < 0)
        {
            grab.setPosition(grab.getPosition() - .005);
        }

        if(gamepad2.left_trigger > 0)
        {
            raiseAutoFull();
        }
        if(gamepad2.left_bumper && gamepad2.x)
        {
            pinchBlock();
        }
        if(gamepad2 .right_trigger > 0)
        {
            lowerAutoArm();
        }



    }

    public void tipOut()
    {
        tipIn.setPosition(.4);
    }
    public void tipIn()
    {
        tipIn.setPosition(.7);
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


    public void raiseAutoArm()
    {
        auto_grab.setPosition(.7);
        sleep(400);
        auto_arm.setPosition(.6);
    }
    public void lowerAutoArm()
    {
        auto_arm.setPosition(.4);
        auto_grab.setPosition(.25);
    }
    public void raiseAutoFull()
    {
        auto_grab.setPosition(.72);
        sleep(100);
        auto_arm.setPosition(.7);
    }
    public void pinchBlock()
    {
        auto_arm.setPosition(.25);
        auto_grab.setPosition(.09);
        sleep(100);
    }

}
