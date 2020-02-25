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

        stonePosition = cv.getValue();
        driveTrain.suckOut();
        arm.tipIn();
        driveTrain.PropDriveIMU(-30,.6,imu);
        driveTrain.drive(-12,.4);
        driveTrain.drive(17,.5);
        driveTrain.suckOff();
        imu.proportionalIMU(90,false);


    }
}
