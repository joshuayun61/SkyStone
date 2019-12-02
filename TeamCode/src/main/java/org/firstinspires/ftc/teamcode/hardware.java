package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class hardware extends LinearOpMode {

    DcMotor FL,FR,BR,BL, Slide;
    Servo intake;
    BNO055IMU imu;

    @Override
    public void runOpMode() {}

    /**
     * Passes the parameters from autonomous so to have a unified telemetry and hardware mapping stemming from autonomous/teleop
     * @param telemetry passing autonomous/teleop telemetry obj because that is the only one that can be used with the program while the autonomous/teleop program is running.
     * @param hardwareMap passing autonomous/teleop hardware mapping obj because that is the only one that can be used with the program while the autonomous/teleop program is running.
     */
    public hardware(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initializes hardwareMapping for the motors in the program, and reverses the correct ones (Left in this case)
     */
    public void setupMotors () {
        //Hardware Map with port on rev hub
        FL = hardwareMap.get(DcMotor.class, "FL"); // 1 fl
        FR = hardwareMap.get(DcMotor.class, "BR"); // 3 fr
        BL = hardwareMap.get(DcMotor.class, "BL"); // 0 bl
        BR = hardwareMap.get(DcMotor.class, "FR"); // 2 Br
        //Slide = hardwareMap.get(DcMotor.class, "Slide");

        //Motor Reversals
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Setup IMU (Inertial Mass Unit) with the correct parameters and hardwareMapping for use in the autnomous commands.
     */
    public void setupIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false; //what does this do?
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /**
     * Setting up the hardware for the servos
     */
    public void setupServos() {
        intake = hardwareMap.get(Servo.class, "intake");
    }
}
