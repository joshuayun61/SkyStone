package org.firstinspires.ftc.teamcode.Regionals_Code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "REpo Zucc")
public class Stones_Only extends LinearOpMode {
    int stonePosition = -100;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1, true);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        imu.imuSetup();


        waitForStart();


        driveTrain.openRepos();
        driveTrain.drive(30,.7);
        driveTrain.strafe(-6,.7);
        driveTrain.drive(3,.3);
        driveTrain.closeRepos();
        sleep(400);
        driveTrain.drive(-25,.7);
        imu.proportionalIMU(-90, true);
        driveTrain.openRepos();
        driveTrain.drive(-3,.7);
        driveTrain.strafe(10,.7);
        arm.openGrabber();
        driveTrain.suckIn();


    }
}
