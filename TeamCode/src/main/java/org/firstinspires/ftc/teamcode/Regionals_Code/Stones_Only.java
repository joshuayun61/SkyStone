package org.firstinspires.ftc.teamcode.Regionals_Code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "Stones Only")
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
        driveTrain.imuStrafe(25,.35,imu);
        switch (stonePosition)
        {
            case(0):
                driveTrain.drive(-7,.6);
                break;
            case(1):
                driveTrain.drive(-3,.8);
                break;
            case(2):
                driveTrain.drive(7,.6);
                break;
        }
        driveTrain.strafe(2,.6);
        arm.raiseAutoArm();
        sleep(100);
        driveTrain.strafe(-6,.6);
        switch (stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(-40,.6,imu);
                break;
            case(1):
                break;
            case(2):
                driveTrain.PropDriveIMU(-50,.6, imu);
                break;
        }
        arm.raiseAutoArm();


    }
}
