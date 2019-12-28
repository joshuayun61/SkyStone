package org.firstinspires.ftc.teamcode.Robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class IMU extends LinearOpMode {

    DcMotor FL,FR,BL,BR, slide;
    BNO055IMU imu;
    Orientation angles, originalAngles;
    Acceleration gravity;
    DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {}

    public IMU(Telemetry telemetry, HardwareMap hardwareMap, DriveTrain myDriveTrain) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveTrain = myDriveTrain;
        FL = driveTrain.FL;
        FR = driveTrain.FR;
        BL = driveTrain.BL;
        BR = driveTrain.BR;
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
    public void resetAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Checks which direction to move in,
     */
    public void turnToAngle(int angle, double power) {

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

        if(angle == 180 || angle == 0)
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
                left =  -power;
                right = power;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < angle) {}

                if(currentRelativeAngle() <= angle-.5)
                {
                    left =  .15;
                    right = -.15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle) {}
                }
                // FOR SOME UNKNOWN REASON, we need to check the angle after turning twice,
                // to end the 1/7 chance of executing the check that was observed before.
                if(currentRelativeAngle() <= angle-.5)
                {
                    left =  .15;
                    right = -.15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle) {}
                }
                if(currentRelativeAngle() >= angle+1)
                {
                    left = -.12;
                    right = .12;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
                }
                if(currentRelativeAngle() >= angle+1)
                {
                    left =  -.12;
                    right = .12;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
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

                if(currentRelativeAngle() >= angle+1)
                {
                    left =  -.15;
                    right = .15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
                }
                // FOR SOME UNKNOWN REASON, we need to check the angle after turning twice,
                // to end the 1/7 chance of executing the check that was observed before.
                if(currentRelativeAngle() >= angle+1)
                {
                    left =  -.15;
                    right = .15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
                }
                if(currentRelativeAngle() <= angle-1)
                {
                    left =  .12;
                    right = -.12;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle-.5) {}
                }
                if(currentRelativeAngle() <= angle-1)
                {
                    left =  .15;
                    right = -.15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle-.5) {}
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

        //and another round of checks:
        if(angle > 0) {
            if(currentRelativeAngle() >= angle+1)
            {
                left =  -.15;
                right = .15;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() > angle+.5) {}
            }

            if(currentRelativeAngle() <= angle)
            {
                left =  .125;
                right = -.125;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < angle-.5) {}
            }
        }
        else{

            if(currentRelativeAngle() >= angle+1)
            {
                left =  -.15;
                right = .15;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() > angle+.5) {}
            }
            if(currentRelativeAngle() <= angle-1)
            {
                left =  .12;
                right = -.12;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < angle-.5) {}
            }

        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final FINAL Heading", currentRelativeAngle());

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
