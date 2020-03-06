package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends LinearOpMode {

    public DcMotor Slide;

    public Servo Intake;

    public Servo cap;

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



        spin = hardwareMap.get(Servo.class, "spin");
        grab = hardwareMap.get(Servo.class, "grab");

        tipIn = hardwareMap.get(Servo.class, "tip");

        cap = hardwareMap.get(Servo.class, "cap");

        auto_arm = hardwareMap.get(Servo.class, "auto_arm");
        auto_grab = hardwareMap.get(Servo.class, "auto_grab");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setDirection(DcMotor.Direction.REVERSE);
    }

    public Arm(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }
    public void reportActive()
    {
        ElapsedTime etime = new ElapsedTime();
        while(etime.time() < 5)
        {
            telemetry.addData("Arm is active", etime.time());
            telemetry.update();
        }
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
            spin.setPosition(.05);
            grab.setPosition(.1);
            while (Slide.getCurrentPosition() > 3) {
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

    public  void autoDrop()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        closeGrabber();
        sleep(100);
        while(Slide.getCurrentPosition() < 1390)
        {
            Slide.setPower(1);
        }
        Slide.setPower(0);
        spinOut();
        sleep(700);
        //while for servo
        while(Slide.getCurrentPosition() > 300)
        {
            Slide.setPower(-1);
        }
        Slide.setPower(0);
        grab.setPosition(.8);
        spinIn();
        sleep(1750);
        while(Slide.getCurrentPosition() > 2)
        {
            Slide.setPower(-1);
        }
        Slide.setPower(0);
        closeGrabber();
    }

    public void readyToPlace()
    {
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setTargetPosition(1390);
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

        if(Slide.getCurrentPosition() < -5)
        {
            home();
        }
        if(gamepad2.left_bumper)
        {
            cap.setPosition(0);
        }
        if(gamepad2.right_bumper)
        {
            cap.setPosition(.3);
        }

        //spin out
        if(gamepad2.x)
        {
            spinOut();
        }
        //spin in
        if(gamepad2.y)
        {
            spinIn();
        }
        //open grabber
        if(gamepad2.a)
        {
            openGrabber();
        }
        //close grabber
        if(gamepad2.b)
        {
            closeGrabber();
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
            tipOut();
        }
        if(gamepad2.dpad_right) //rotate in
        {
            tipInward();
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
            raise();
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
    public void tipInward()
    {
        tipIn.setPosition(.7);
    }
    public void closeGrabber()
    {
        grab.setPosition(0);
        tipIn.setPosition(.4);
    }
    public void openGrabber()
    {
        grab.setPosition(.5);
    }
    public void spinIn()
    {
        spin.setPosition(.05);
    }
    public void spinOut()
    {
        spin.setPosition(.78);
    }





    public void raiseAutoArm()
    {
        auto_grab.setPosition(.7);
        auto_arm.setPosition(.6);
    }
    public void lowerAutoArm()
    {
        auto_arm.setPosition(.52);
        auto_grab.setPosition(.25);
    }
    public void grabAndRaise()
    {
        auto_grab.setPosition(.45);
        auto_arm.setPosition(.35);
        sleep(150);
        auto_grab.setPosition(.6);
        sleep(100);
        auto_arm.setPosition(.6);
    }
    public void dropBlock()
    {
        auto_arm.setPosition(.55);
        auto_grab.setPosition(.25);
    }

    public void raiseAutoFull()
    {
        auto_grab.setPosition(.3);
        sleep(100);
        auto_arm.setPosition(.7);
    }
    public void raise()
    {
        auto_grab.setPosition(.56);
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
