package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "autonomous", group = "Linear OpMode")

public class autonomous extends LinearOpMode
{

    /*
     * Hardware variables
     */
    BNO055IMU imu;
    DcMotor FL,FR,BL,BR;
    Servo grabber;

    /*
     * Gyroscope Data
     */
    double facing;
    Orientation angles;
    Acceleration gravity;


    @Override
    public void runOpMode() throws InterruptedException {

        tfod vision  = new tfod();

        IMUSetupAndMapping();

        hardwareMapping();


        while(!opModeIsActive()) {
            vision.runTfod();
        }

        if (opModeIsActive()) {
            vision.stopTfod();
            while (opModeIsActive()) {
                if (vision.skyStoneLocation == 1) {

                } else if (vision.skyStoneLocation == 2) {

                } else {

                }
                telemetry.update();
            }
        }
    }

    private void skyStoneLeft() {

    }

    private void skyStoneRight() {

    }

    private void skyStoneMiddle() {

    }

    private void IMUSetupAndMapping() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false; //what does this do?
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.initialize(parameters);

        //Check IMU calibration
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

    }

    private void IMUResetAngle() {
        //This resets the angles of the IMU, eg: First Angle is now 0 wherever it is
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        facing = 0;
    }

    private double snapshotCurrentAngle() {
        //This creates a new 'snapshot' of the current angles, to be checked against the resetted angular orientatoin
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - angles.firstAngle;

        if (changeInAngle < -180){

            changeInAngle += 360;

        } else if (changeInAngle > 180){

            changeInAngle -= 360;

        }

        facing += changeInAngle;

        angles = newAngles;

        return facing;
    }


    private void forwardEncoders() {

    }

    private void hardwareMapping() {

        //Initializing the hardware variables to the configuration mappings in the App
        FL = hardwareMap.get(DcMotor.class, "FL"); // 1 fl
        FR = hardwareMap.get(DcMotor.class, "BR"); // 3 fr
        BL = hardwareMap.get(DcMotor.class, "BL"); // 0 bl
        BR = hardwareMap.get(DcMotor.class, "FR"); // 2 Br

        //Motor Reversals so positive is the same rotation for all motors.
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
    }





}
