package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class TeleOpCommands extends LinearOpMode {

    private DcMotor FL, FR, BR, BL, Slide;
    private BNO055IMU imu;
    private Orientation angles;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    TeleOpCommands(Telemetry telemetry, HardwareMap hardwaremap, Gamepad inputGamepad1, Gamepad inputGamepad2) {
        gamepad1 = inputGamepad1;
        gamepad2 = inputGamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwaremap;
        Hardware hardware = new Hardware(telemetry, hardwaremap);
        hardware.setup();

        FL = hardware.FL;
        FR = hardware.FR;
        BL = hardware.BL;
        BR = hardware.BR;
        Slide = hardware.Slide;

        imu = hardware.imu;
    }

    @Override
    public void runOpMode() {}


    void mecanum() {

        double drive    = -gamepad1.right_stick_y;
        double strafe   = gamepad1.right_stick_x;
        double spin     = gamepad1.left_stick_x;

        if(gamepad1.right_bumper)
        {

            FL.setPower(-drive + strafe - spin);
            FR.setPower(drive - strafe - spin);
            BL.setPower(-drive - strafe - spin);
            BR.setPower(drive + strafe - spin);

        }
        else
        {
            FL.setPower((-drive + strafe - spin)/3);
            FR.setPower((drive - strafe - spin)/3);
            BL.setPower((-drive - strafe -
                    spin)/3);
            BR.setPower((drive + strafe - spin)/3);
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
    }

    void turn180() {
        if(gamepad1.y)
        {
            turnToAngle(180,.7);
        }
    }


    /**
     * Resets the measurments so that they are starting from 0, useful if you only want to turn relative to current location,
     * rather than having to remeber the original location of the robot.
     */
    private void resetAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Checks which direction to move in,
     */
    private void turnToAngle(int angle, double power) {

        //          0
        //  90              -90
        //         180

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAngle();

        telemetry.addData("After Reset: ", currentRelativeAngle());
        telemetry.addData("Heading input: ", angle);

        double left, right;

        //added a special to check if the entered angle is 180,
        //because this will be the most used angle, so it must work best

        if(angle == 180)
        {
            left =  power;
            right = -power;
            FL.setPower(left);
            BL.setPower(left);
            FR.setPower(right);
            BR.setPower(right);
            while (angle > currentRelativeAngle() && currentRelativeAngle() >= 0) {}
            if(currentRelativeAngle() < 0)
            {
                left =  -.2;
                right = .2;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < 0) {}
            }
        } else {

            //Turns based on angle set at desired power while currentAngle is not matching the target angle.
            if (angle < 0) {
                left = -power;
                right = power;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (angle < currentRelativeAngle()) {}

                if(currentRelativeAngle() < angle-1)
                {
                    left =  .2;
                    right = -.2;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle-1) {}
                }
            }
            else if (angle > 0) {
                left =  power;
                right = -power;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (angle > currentRelativeAngle()) {}

                if(currentRelativeAngle() > angle+1)
                {
                    left =  -.2;
                    right = .2;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+1) {}
                }
            }
            else {
                return;
            }
        }

        //Set power to 0 after the turning
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final Heading", currentRelativeAngle());

        resetAngle();

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Gets a new orientation, finds the change in angle from the older orientation.
     * @return Returns the change in angle from the starting point of when the angles were last reset (During reset it is set to 0)
     */
    private double currentRelativeAngle() {
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - angles.firstAngle;

        if (changeInAngle < -180){
            changeInAngle += 360;
        } else if (changeInAngle > 180){
            changeInAngle -= 360;
        }

        return changeInAngle;
    }
}
