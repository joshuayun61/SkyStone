package org.firstinspires.ftc.teamcode.Regionals_Code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.armThread;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "bruh regionals")
@Disabled
public class Only_Stones_RED extends LinearOpMode {
    int stonePosition = -100;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1, true);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        armThread dropThread = new armThread(telemetry, arm);
        OpenCV cv = new OpenCV(telemetry, hardwareMap);
        imu.imuSetup();
        cv.setupWebCam();
        cv.getValue();
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Stone Position", cv.getValue());
            telemetry.update();
            stonePosition = cv.getValue();
        }

        waitForStart();

        //open grabber, move out tipper, turn on zucc
        arm.grab.setPosition(0);
        arm.spinIn();
        driveTrain.suckIn();
        arm.tipOut();
        //drive to blocks
        driveTrain.PropDriveIMU(-28, .7, imu);
        switch (stonePosition) {
            case (0):
                break;
            case (1):
                driveTrain.strafe(5, .7);
                break;
            case (2):
                driveTrain.strafe(13, .7);
                break;
        }
        if (stonePosition != 1)
            driveTrain.drive(-13, .4);
        else
            driveTrain.drive(-15, .4);

        if (stonePosition == 2)
            driveTrain.drive(18, .65);
        else if (stonePosition == 0)
            driveTrain.drive(15, .65);
        else
            driveTrain.drive(18, .65);

        driveTrain.suckOff();
        //imu.proportionalIMU(-90,false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(-45,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-45,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(-36,.7,imu);
                break;
        }
        driveTrain.suckOut();
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(68,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(72,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(60,.7,imu);
                break;
        }
        driveTrain.suckOff();

        if(stonePosition == 0) {
          //  imu.proportionalIMU(0, false);
        }
        else
            driveTrain.spin(800,.6);
        driveTrain.suckIn();
        arm.openGrabber();
        driveTrain.drive(-10,.5);
        driveTrain.drive(15, .6);
        arm.tipInward();
        sleep(400);
        arm.closeGrabber();
        driveTrain.suckOff();
        //imu.proportionalIMU(-90,false);


        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(-76,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-73,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(-69,.7,imu);
                break;
        }
        driveTrain.drive(23,.5);


    }
}
