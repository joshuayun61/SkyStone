package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "CV AUTO (RED DEPOT)")
public class CVAuto extends LinearOpMode {

    int stonePosition = -100;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        OpenCV cv = new OpenCV(telemetry, hardwareMap);
        imu.imuSetup();
        cv.setupWebCam();
        cv.getValue();
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Stone Position", cv.getValue());
            telemetry.update();
            stonePosition = cv.getValue();
        }

        //positive drive is toward intake
        //positive strafe is toward auto grabber
        waitForStart();

        arm.lowerAutoArm();

        if(stonePosition == 2)
        {
            driveTrain.drive(8,.55);
        }

        driveTrain.imuStrafe(25,.35, imu);
        switch(stonePosition)
        {
            case(0) :
                driveTrain.drive(-8,.55);
            case(1):
                break;
        }
        //Grab Skystone
        driveTrain.strafe(2,.3);
        arm.raiseAutoArm();
        driveTrain.strafe(-5,.45);
        imu.proportionalIMU(0,false);
        //Drive to Foundation
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(-80, .6, imu);
                break;
        }
        driveTrain.strafe(5,.4);
        //Deposit Skystone and Resposition
        arm.dropBlock();
        sleep(200);
        arm.raiseAutoArm();
        arm.openRepos();
        imu.proportionalIMU(90,false);
        driveTrain.drive(4,.4);
        arm.closeRepos();
        sleep(700);
        driveTrain.drive(-10,.4);




    }
}