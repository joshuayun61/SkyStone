package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleopBackUp")
public class TeleopBackup extends LinearOpMode
{

    DcMotor FL, FR, BR, BL, Slide;

    BNO055IMU imu;
    Orientation angles, originalAngles;
    Acceleration gravity;

    public void runOpMode() {

        Hardware hardware = new Hardware(telemetry, hardwareMap);
        hardware.setup();

        FL = hardware.FL;
        FR = hardware.FR;
        BL = hardware.BL;
        BR = hardware.BR;
        Slide = hardware.Slide;

        imu = hardware.imu;

        waitForStart();

        while(opModeIsActive()) {

            double drive    = gamepad1.right_stick_y;
            double strafe   = -gamepad1.right_stick_x;
            double spin     = -gamepad1.left_stick_x;

            if(gamepad1.right_bumper)
            {
                FL.setPower((drive + strafe + spin)/3);
                FR.setPower((drive - strafe - spin)/3);
                BL.setPower((drive - strafe + spin)/3);
                BR.setPower((drive + strafe - spin)/3);
            }
            else
            {
                FL.setPower(drive + strafe + spin);
                FR.setPower(drive - strafe - spin);
                BL.setPower(drive - strafe + spin);
                BR.setPower(drive + strafe - spin);
            }

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

        }
    }
}
