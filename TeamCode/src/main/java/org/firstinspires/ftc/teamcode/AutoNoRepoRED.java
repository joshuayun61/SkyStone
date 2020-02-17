package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;

@Autonomous(name = "Autonomous Red NO REPO")
@Disabled
public class AutoNoRepoRED extends LinearOpMode {


    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain);
        sense.setupTfod();
        imu.imuSetup();

        waitForStart();












    }
}
