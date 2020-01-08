package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ImuV2 {

    private static Telemetry telemetry;
    private static HardwareMap hardwareMap;
    private Gamepad gamepad1;

    private BNO055IMU imu;

    private Orientation initialOrientation;

    public ImuV2 (Telemetry importTelemetry, HardwareMap importHardwareMap, Gamepad gamepad) {

        telemetry = importTelemetry;
        hardwareMap = importHardwareMap;
        gamepad1 = gamepad;

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
        initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Puts the initial heading into 0-360 form, more usable
        initialOrientation.firstAngle += 360;
        initialOrientation.firstAngle %= 360;
    }

    private float currentHeading() {

        //          0
        //  90              -90
        //     179.9   -179.9

        //Map of orientation heading

        Orientation currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        return currentOrientation.firstAngle;
    }

    private float findError(int targetAngle) {
        return  currentHeading() - targetAngle;
    }

    public double pidPower(int targetAngle) {

        findError(targetAngle);

        if ()

        return 0;
    }
}
