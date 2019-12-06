package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUCommands extends LinearOpMode {

    Hardware hardware;
    private DcMotor Slide;
    private BNO055IMU imu;
    private Orientation angles;
    DcMotor FL, FR, BL, BR;
    DcMotor driveMotors[];

    public IMUCommands(Telemetry telemetry, HardwareMap hardwaremap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwaremap;
        hardware = new Hardware(telemetry, hardwaremap);
        hardware.setup();

        DcMotor[] motorsTemp = {hardware.FL, hardware.FR, hardware.BL, hardware.BR};
        driveMotors = motorsTemp;

        Slide = hardware.Slide;

        imu = hardware.imu;

        FL = hardware.FL;
        FR = hardware.FR;
        BL = hardware.BL;
        BR = hardware.BR;
    }



    @Override
    public void runOpMode() throws InterruptedException {}

    /**
     * Resets the measurments so that they are starting from 0, useful if you only want to turn relative to current location,
     * rather than having to remeber the original location of the robot.
     */
    public void resetAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Checks which direction to move in,
     */
    public void turnToAngle(int angle, double power) {

        //          0
        //  90              -90
        //         180

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

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

        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * Gets a new orientation, finds the change in angle from the older orientation.
     * @return Returns the change in angle from the starting point of when the angles were last reset (During reset it is set to 0)
     */
    public double currentRelativeAngle() {
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
