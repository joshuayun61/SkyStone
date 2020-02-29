package org.firstinspires.ftc.teamcode.Regionals_Code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
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
        driveTrain.drive(18,.5);
        arm.tipInward();
        sleep(200);
        driveTrain.suckOff();
        arm.closeGrabber();
        imu.proportionalIMU(-90,false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(75,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(85,.4,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(90,.4,imu);
                break;
        }
        driveTrain.spin(-730,.7);
        arm.openRepos();
        driveTrain.drive(7,.45);
        arm.autoDrop();
        arm.home();
        driveTrain.drive(-6,.6);
        imu.proportionalIMU(-90,false);
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(-85,.7,imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-85,.4,imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(-90,.4,imu);
                break;
        }



    }
}
