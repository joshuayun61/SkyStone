package org.firstinspires.ftc.teamcode.Regionals_Code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "Stones Only")
@Disabled
public class Stones_Only extends LinearOpMode {
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


        stonePosition = cv.getValue();
        arm.openRepos();
        arm.lowerAutoArm();
        driveTrain.imuStrafe(24, .37, imu);
        switch (stonePosition) {
            case (0):
                driveTrain.drive(-6, .6);
                break;
            case (1):
                break;
            case (2):
                driveTrain.drive(7, .6);
                break;
        }
        driveTrain.strafe(2, .6);
        arm.grabAndRaise();
        driveTrain.strafe(-7, .6);
        switch (stonePosition) {
            case (0):
                driveTrain.PropDriveIMU(-48, .5, imu);
                break;
            case (1):
                driveTrain.PropDriveIMU(-53, .5, imu);
                break;
            case (2):
                driveTrain.PropDriveIMU(-60, .5, imu);
                break;
        }
        arm.dropBlock();
        sleep(100);
        arm.lowerAutoArm();
        imu.proportionalIMU(0, false);
        switch (stonePosition) {
            case (0):
                driveTrain.PropDriveIMU(75, .5, imu);
                break;
            case (1):
                driveTrain.PropDriveIMU(80, .5, imu);
                break;
            case (2):
                break;
        }
        driveTrain.strafe(8, .6);
        arm.grabAndRaise();
        driveTrain.strafe(-13, .6);
        switch (stonePosition) {
            case (0):
                driveTrain.PropDriveIMU(-75, .6, imu);
                // 2nd skystone
                arm.dropBlock();
                sleep(100);
                arm.lowerAutoArm();
                imu.proportionalIMU(0,false);

                driveTrain.PropDriveIMU(62, .7, imu);
                //3rd stone
                driveTrain.strafe(12, .6);
                arm.grabAndRaise();
                driveTrain.strafe(-10, .6);
                driveTrain.PropDriveIMU(-62,.7,imu);

        }

    }
}
