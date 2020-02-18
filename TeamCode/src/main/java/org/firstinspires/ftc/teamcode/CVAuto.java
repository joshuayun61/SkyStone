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
                driveTrain.drive(-7,.55);
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
            case(1):
                driveTrain.PropDriveIMU(-88, .6, imu);
                break;
            case(2) :
                driveTrain.PropDriveIMU(-93, .6, imu);
                break;
        }
        driveTrain.strafe(6,.4);
        //Deposit 1st Skystone
        arm.dropBlock();
        sleep(200);
        arm.raiseAutoArm();
        driveTrain.strafe(-3,.4);
        //Go back for 2nd
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(105, .7, imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(108, .7, imu);
                break;

        }
        //Grab 2nd Skystone
        arm.lowerAutoArm();
        driveTrain.imuStrafe(8,.35,imu);
        arm.raiseAutoArm();
        driveTrain.strafe(-8,.4);
        imu.proportionalIMU(0,false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(-105, .6, imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-90, .6, imu);
                break;
//            case(2) :
//                driveTrain.PropDriveIMU(-93, .6, imu);
//                break;
        }
        driveTrain.strafe(6,.4);
        //Deposit 2nd Skystone
        arm.dropBlock();
        sleep(200);
        arm.raiseAutoArm();
        arm.openRepos();
        driveTrain.drive(3,.4);
        imu.proportionalIMU(90,false);
        driveTrain.drive(4,.3);
        arm.closeRepos();
        sleep(700);
        driveTrain.drive(-20,.4);
   //     imu.proportionalIMU(180, true);




    }
}