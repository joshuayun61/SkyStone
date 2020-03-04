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

@Autonomous(name = "Stones Zucc")
public class Stone_Zucc extends LinearOpMode {
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

      //  stonePosition = cv.getValue();
        //open grabber, move out tipper, turn on zucc
        arm.grab.setPosition(.5);
        driveTrain.suckIn();
        arm.tipOut();
        //drive to blocks
        driveTrain.PropDriveIMU(-29,.7,imu);
        switch (stonePosition)
        {
            case(0):
                driveTrain.strafe(-3,.7);
                break;
            case(1):
                break;
            case(2):
                driveTrain.strafe(6,.7);
                break;
        }
        driveTrain.drive(-12,.4);
        driveTrain.drive(18,.65);
        arm.tipInward();
        sleep(350);
        driveTrain.suckOff();
        arm.closeGrabber();
        imu.proportionalIMU(-90,false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(76,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(85,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(90,.4,imu);
                break;
        }

        dropThread.start();
        driveTrain.spin(-680,.55);
        arm.openRepos();
        driveTrain.drive(14,.4);
        arm.closeRepos();
        sleep(500);
        driveTrain.drive(-19,.6);
        imu.proportionalIMU(-90,true);
        arm.openRepos();

        switch (stonePosition){
            case(0):
                driveTrain.PropDriveIMU(-83,.73,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-93,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(-93,.4,imu);
                break;
        }
        arm.home();
        driveTrain.spin(500,.5);
        driveTrain.suckIn();
        arm.openGrabber();
        driveTrain.drive(-12,.38);
        driveTrain.drive(16,.65);
        arm.tipInward();
        sleep(350);
        driveTrain.suckOff();
        arm.closeGrabber();
        imu.proportionalIMU(-90, false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(63,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(68,.7,imu);
                break;

        }
        driveTrain.drive(-7,.5);



    }
}
