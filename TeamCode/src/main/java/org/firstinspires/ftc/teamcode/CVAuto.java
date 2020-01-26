package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "CV AUTO ")
public class CVAuto extends LinearOpMode {

    int stonePosition = -100;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        //  DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        // TensorSense sense = new TensorSense(telemetry, hardwareMap);
        //  IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        //   Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain);
        // sense.setupTfod();
        OpenCV cv = new OpenCV(telemetry, hardwareMap);
        //imu.imuSetup();
        cv.setup();
        cv.getValue();
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Stone Position", cv.getValue());
            telemetry.update();
            stonePosition = cv.getValue();
        }
        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Stone Position", cv.getValue());
            telemetry.update();
            stonePosition = cv.getValue();
        }

    }
}