package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ConfigurationTest extends OpMode {

    DcMotor FL,FR,BR,BL, Slide;
    Servo intake;

    public void init() {

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "FR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        Slide = hardwareMap.get(DcMotor.class, "Slide");
        intake = hardwareMap.get(Servo.class, "intake");

    }

    public void loop() {

        FL.setPower(gamepad1.left_stick_y);
        BL.setPower(gamepad1.left_stick_y);
        FR.setPower(gamepad1.right_stick_y);
        BR.setPower(gamepad1.right_stick_y);

    }

}
