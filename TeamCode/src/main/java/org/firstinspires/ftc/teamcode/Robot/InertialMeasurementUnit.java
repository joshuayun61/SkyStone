package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot12382;

public class InertialMeasurementUnit {

    BNO055IMU imu;

    public static final double Pi = Math.PI;

    public static final double minumumPower = 0.1;

    public void imuSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false; //what does this do?
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = Robot12382.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double calculateError(double target) {

        double error =  imu.getAngularOrientation().firstAngle - target;

        if (error < -Pi) {
            error += 2 * Pi;
        }
        else if (error > Pi) {
            error -= 2*Pi;
        }

        return error/Pi;
    }
}