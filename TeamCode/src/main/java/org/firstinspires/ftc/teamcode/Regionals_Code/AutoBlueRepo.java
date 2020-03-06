package org.firstinspires.ftc.teamcode.Regionals_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;

@Autonomous(name = "Autonomous BLUE REPO (Park Far)")
public class AutoBlueRepo extends LinearOpMode {

    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        imu.imuSetup();

        waitForStart();

        driveTrain.openRepos();
        driveTrain.spline(.5,35,135);
        driveTrain.drive(12,.5);
        driveTrain.closeRepos();
        sleep(350);
        driveTrain.drive(-30,.5);
        imu.proportionalIMU(90,true);
        driveTrain.openRepos();
        driveTrain.spline(.5,35,315);
        driveTrain.drive(-15,.5);
    }
}
