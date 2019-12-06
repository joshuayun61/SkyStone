package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class TeleOpCommands extends LinearOpMode {

    private IMUCommands imu;

    private DcMotor Slide;
    private Orientation angles;
    DcMotor FL, FR, BL, BR;
    Servo intake;

    private DcMotor[] driveMotors = null;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    TeleOpCommands(Telemetry telemetry, HardwareMap hardwaremap, Gamepad inputGamepad1, Gamepad inputGamepad2) {

        gamepad1 = inputGamepad1;
        gamepad2 = inputGamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwaremap;
        Hardware hardware = new Hardware(telemetry, hardwaremap);
        hardware.setup();

        DcMotor[] motorsTemp = {hardware.FL, hardware.FR, hardware.BL, hardware.BR};
        driveMotors = motorsTemp;

        Slide = hardware.Slide;
        FL = hardware.FL;
        FR = hardware.FR;
        BL = hardware.BL;
        BR = hardware.BR;

        intake = hardware.intake;

        IMUCommands tempImu = new IMUCommands(telemetry, hardwaremap);

        imu = tempImu;
    }

    @Override
    public void runOpMode() {}


    void mecanum() {

        double drive    = gamepad1.right_stick_y;
        double strafe   = gamepad1.right_stick_x;
        double spin     = gamepad1.left_stick_x;

        if(gamepad1.right_bumper)
        {

            FL.setPower(drive - strafe - spin);
            FR.setPower(drive + strafe + spin);
            BL.setPower(drive + strafe - spin);
            BR.setPower(drive - strafe + spin);

        }
        else
        {
            FL.setPower((drive - strafe - spin)/3);
            FR.setPower((drive + strafe + spin)/3);
            BL.setPower((drive + strafe - spin)/3);
            BR.setPower((drive - strafe + spin)/3);
        }
    }

    void slideMotor() {

        if (gamepad1.a) {
            Slide.setTargetPosition(Slide.getCurrentPosition() + 3);
            Slide.setPower(.4);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (gamepad1.b) {
            Slide.setTargetPosition(Slide.getCurrentPosition() - 3);
            Slide.setPower(.4);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Position", Slide.getCurrentPosition());
    }

    void openIntake() {
        if (gamepad1.a) {
            intake.setPosition(0.7);
        }
    }
    void closeIntake() {
        if (gamepad1.b) {
            intake.setPosition(0);
        }
    }

    void turn180() {
        if(gamepad1.y)
        {
            imu.turnToAngle(180,.7);
        }
    }
}
