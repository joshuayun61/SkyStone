package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;

@Autonomous(name = "Red Bridge Far")
public class AutoRepoRedBridge2 extends LinearOpMode {


    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
     //   TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        // sense.setupTfod();
        imu.imuSetup();

        waitForStart();

       driveTrain.drive(-24, .6);
       arm.openRepos();
       driveTrain.strafe(15, .3);
       driveTrain.drive(-10,.3);
       arm.closeRepos();
       sleep(1000);
       driveTrain.drive(36,.6);
       imu.proportionalIMU(-90,true);
       arm.openRepos();
        driveTrain.strafe(-16, .3);
        driveTrain.drive(33,.3);


        //pull foundation










    }
}
