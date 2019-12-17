package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class InetialMUnit  extends LinearOpMode {

    DcMotor FL, FR, BL, BR, slide;
    BNO055IMU imu;
    Orientation angles, originalAngles;
    Acceleration gravity;
    DriveTrain driveTrain;
    DcMotor[] motors = new DcMotor[4];


    @Override
    public void runOpMode() throws InterruptedException {
    }

    public InetialMUnit(Telemetry telemetry, HardwareMap hardwareMap, DriveTrain myDriveTrain) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveTrain = myDriveTrain;
        driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        FL = driveTrain.FL;
        FR = driveTrain.FR;
        BL = driveTrain.BL;
        BR = driveTrain.BR;
        DcMotor[] tempMotors = {FR, FR, BL, BR};
        motors = tempMotors;
    }

    public void imuSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false; //what does this do?
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
     * Resets the measurments so that they are starting from 0, useful if you only want to turn relative to current location,
     * rather than having to remeber the original location of the robot.
     */
    public void resetAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void turnToAngle(double power, int angle) {

        //          0
        //  90              -90
        //         180


        resetAngle();

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("After Reset: ", currentRelativeAngle());
        telemetry.addData("Heading input: ", angle);

        double left, right;

        if (angle < 0) {
            left = -power;
            right = power;
            FL.setPower(left);
            BL.setPower(left);
            FR.setPower(right);
            BR.setPower(right);
            while (angle < currentRelativeAngle()) {
            }

            if (currentRelativeAngle() < angle - 1) {
                left = .2;
                right = -.2;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < angle - 1) {
                }
            }
        } else if (angle > 0) {
            left = power;
            right = -power;
            FL.setPower(left);
            BL.setPower(left);
            FR.setPower(right);
            BR.setPower(right);
            while (angle > currentRelativeAngle()) {
            }

            if (currentRelativeAngle() > angle + 3) {
                left = -.15;
                right = .15;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() > angle + 3) {
                }
            }
            if (currentRelativeAngle() > angle + 3) {
                left = -.15;
                right = .15;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() > angle + 3) {
                }
            }
        } else {
            return;
        }


    //Set power to 0 after the turning
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final Heading", currentRelativeAngle());



        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


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
