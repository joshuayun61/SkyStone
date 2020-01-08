package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrainV2 {

    private static Telemetry telemetry;
    private static HardwareMap hardwareMap;
    private Gamepad gamepad1;

    private DcMotor FL, FR, BL, BR;
    private DcMotor[] motors = new DcMotor[4];

    private final float Ku = 2f;
    private float Kp = Ku/2;

    public DriveTrainV2 (Telemetry importTelemetry, HardwareMap importHardwareMap, Gamepad gamepad) {

        telemetry = importTelemetry;
        hardwareMap = importHardwareMap;
        gamepad1 = gamepad;

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        motors[0] = FL;
        motors[1] = FR;
        motors[2] = BL;
        motors[3] = BR;
    }

    public void mecanumDrive() {

        double drive    = gamepad1.right_stick_y;
        double strafe   = -gamepad1.right_stick_x;
        double spin     = gamepad1.left_stick_x;

        double flPower = drive - strafe - spin;
        double frPower = drive + strafe + spin;
        double blPower = drive + strafe - spin;
        double brPower = drive - strafe + spin;

        double[] motorPowers = {flPower, frPower, blPower, brPower};

        if (gamepad1.left_bumper) {
            for (int i=0; i < motorPowers.length; i++) {
                motorPowers[i] /= 6;
            }
        }
        else if (!gamepad1.right_bumper) {
            for (int i=0; i < motorPowers.length; i++) {
                motorPowers[i] /= 3;
            }
        }

        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);

        if(gamepad1.right_trigger > 0)
        {
            slowStrafe("right");
        }
        if(gamepad1.left_trigger > 0)
        {
            slowStrafe("left");
        }
    }

    private void slowStrafe(String direction) {
        if (direction.toLowerCase().equals("right")) {
            FL.setPower(.3);
            FR.setPower(-.3);
            BR.setPower(.3);
            BL.setPower(-.3);
        }
        else if (direction.toLowerCase().equals("left")) {
            FL.setPower(-.3);
            FR.setPower(.3);
            BR.setPower(-.3);
            BL.setPower(.3);
        }
    }



    private int inchesToTicks(double distance) { return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6)); }
}
