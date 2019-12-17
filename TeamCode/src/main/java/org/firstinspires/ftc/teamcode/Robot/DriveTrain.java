package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain extends LinearOpMode {

    DcMotor FL, FR, BL, BR;

    DcMotor[] motors = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public DriveTrain(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad) {

        gamepad1 = gamepad;

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] tempMotors = {FL, FR, BL, BR};
        motors = tempMotors;
    }

    public void mecanumDrive() {

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
        else if(gamepad1.left_bumper)
        {
            FL.setPower((drive - strafe - spin)/6);
            FR.setPower((drive + strafe + spin)/6);
            BL.setPower((drive + strafe - spin)/6);
            BR.setPower((drive - strafe + spin)/6);
        }
        else
        {
            FL.setPower((drive - strafe - spin)/3);
            FR.setPower((drive + strafe + spin)/3);
            BL.setPower((drive + strafe - spin)/3);
            BR.setPower((drive - strafe + spin)/3);
        }
    }

    public void drive(double distance, double power) {

        int ticks = inchesToTicks(distance);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() - ticks);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("EncoderPosition", FL.getCurrentPosition());
            telemetry.addData("EncoderTarget", ticks);
            telemetry.update();
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void strafe(double distance, double power) {

        int ticks = inchesToTicks(distance);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);


        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("EncoderPosition", FL.getCurrentPosition());
            telemetry.addData("EncoderTarget", ticks);
            telemetry.update();
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public int inchesToTicks(double distance) {

        return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6));
    }

}
