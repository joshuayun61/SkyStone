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

@Autonomous(name = "Stones Zucc")
@Disabled
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
                driveTrain.strafe(7,.7);
                break;
        }
        if(stonePosition != 1)
            driveTrain.drive(-12,.4);
        else
            driveTrain.drive(-14,.4);
        if(stonePosition != 1)
            driveTrain.drive(18,.65);
        else
            driveTrain.drive(20,.65);
        arm.tipInward();
        sleep(350);
        driveTrain.suckOff();
        arm.closeGrabber();
        //imu.proportionalIMU(-90,false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(76,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(85,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(86,.7,imu);
                break;
        }

        dropThread.start();
        driveTrain.spin(-660,.55);
        driveTrain.openRepos();
        driveTrain.drive(14,.4);
        driveTrain.closeRepos();
        sleep(500);
        driveTrain.drive(-20,.6);
        //imu.proportionalIMU(-90,true);
        driveTrain.openRepos();

        switch (stonePosition){
            case(0):
                driveTrain.PropDriveIMU(-83,.73,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-93,.7,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(-97,.7,imu);
                break;
        }
        driveTrain.spin(500,.5);
        driveTrain.suckIn();
        arm.openGrabber();
        driveTrain.drive(-14,.38);
        driveTrain.drive(13,.65);
        arm.tipInward();
        sleep(350);
        driveTrain.suckOff();
        arm.closeGrabber();
        //imu.proportionalIMU(-90, false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(65,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(70,.7,imu);
                break;
            case(2) :
                driveTrain.PropDriveIMU(75,.8,imu);
                break;

        }
        driveTrain.drive(-8,.5);



    }
}
