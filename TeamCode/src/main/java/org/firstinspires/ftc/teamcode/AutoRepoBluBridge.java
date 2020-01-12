package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;

@Autonomous(name = "Autonomous Blue Bridge")
public class AutoRepoBluBridge extends LinearOpMode {


    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain);
        // sense.setupTfod();
        imu.imuSetup();

        waitForStart();

        arm.raisePH(600);

        driveTrain.drive(20,.5);
        driveTrain.strafe(-15,.3);
        driveTrain.reposOpen();
        sleep(200);
        driveTrain.drive(20, .4);
        driveTrain.reposClose();
        sleep(200);
        driveTrain.drive(-30,.4);
        imu.proportionalIMU(90,true);
        driveTrain.reposOpen();
        sleep(200);

        driveTrain.strafe(-10, .4);
        driveTrain.drive(-15,.6);
        arm.ground();
        driveTrain.drive(-15,.6);



        //pull foundation










    }
}
