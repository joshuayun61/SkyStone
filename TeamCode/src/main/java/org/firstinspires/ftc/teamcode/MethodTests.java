package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.AI.armThread;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.IMU;

@Autonomous(name = "Methods Test")
public class MethodTests extends LinearOpMode {

    ElapsedTime etime = new ElapsedTime();

    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1, true);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        armThread dropThread = new armThread(telemetry, arm);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        imu.imuSetup();

        waitForStart();

        driveTrain.PropDriveIMU(30,.7,imu, 90);

    }
}
