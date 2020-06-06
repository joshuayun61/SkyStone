package org.firstinspires.ftc.teamcode.Regionals_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;

@Autonomous(name = "Blue Bridge")
@Disabled
public class AutoRepoBluBridge extends LinearOpMode {


    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        //   TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        // sense.setupTfod();
        imu.imuSetup();

        waitForStart();

        driveTrain.drive(-24, .6);
        driveTrain.openRepos();
        driveTrain.strafe(-10, .3);
        driveTrain.drive(-10,.3);
        driveTrain.closeRepos();
        sleep(1000);
        driveTrain.drive(36,.6);
        //imu.proportionalIMU(90,true);
        driveTrain.openRepos();
        driveTrain.drive(-5,.7);
        driveTrain.strafe(-6, .3);
        driveTrain.drive(40,.3);



        //pull foundation










    }
}
