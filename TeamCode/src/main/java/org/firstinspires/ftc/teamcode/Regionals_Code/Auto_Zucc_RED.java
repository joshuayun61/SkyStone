package org.firstinspires.ftc.teamcode.Regionals_Code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.AI.armThread;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "Stones Zucc RED")
public class Auto_Zucc_RED extends LinearOpMode {
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
        arm.grab.setPosition(.5);
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
                driveTrain.strafe(12, .7);
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

        arm.tipInward();
        sleep(350);
        driveTrain.suckOff();
        arm.closeGrabber();
        imu.proportionalIMU(90,false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(84,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(85,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(76,.7,imu);
                break;
        }

        dropThread.start();
        if(stonePosition == 0) {
            driveTrain.spin(590, .55);
        }
        else
        driveTrain.spin(600,.55);
        driveTrain.openRepos();
        driveTrain.drive(16,.28);
        driveTrain.closeRepos();
        sleep(500);
        driveTrain.drive(-26,.6);
        imu.safeIMU(90,true);
        driveTrain.openRepos();
        if(stonePosition == 0)
        {
            driveTrain.strafe(2,.5);
        }
        else if(stonePosition == 1) {
            driveTrain.strafe(4, .5);
        }
        else
            driveTrain.strafe(6,.5);
        switch (stonePosition){
            case(0):
                driveTrain.PropDriveIMU(-90,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-90,.65,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(-85,.65,imu);
                break;
        }
        dropThread.interrupt();
        if(stonePosition != 0) {
            driveTrain.spin(-320, .5);
        }
        else
        {
            driveTrain.strafe(7,.5);
        }
        arm.openGrabber();
        driveTrain.suckIn();

        if(stonePosition != 0) {
            driveTrain.drive(-14, .38);
            driveTrain.drive(16 , .6);
        }
        else
        {
            driveTrain.drive(-11, .38);
            driveTrain.drive(11, .6);
            driveTrain.strafe(-6,.5);
        }
        arm.tipInward();
        sleep(350);
        driveTrain.suckOff();
        arm.closeGrabber();
        imu.proportionalIMU(90, false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(70,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(65,.7,imu);
                break;
            case(2) :
                driveTrain.PropDriveIMU(58,.8,imu);
                break;

        }
        driveTrain.drive(-10,.5);



    }
}
