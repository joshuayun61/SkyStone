package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "CV AUTO BLUE ")
public class CVBlue extends LinearOpMode {

    int stonePosition = -100;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        // TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        //   Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain);
        // sense.setupTfod();
        OpenCV cv = new OpenCV(telemetry, hardwareMap);
        imu.imuSetup();
      //'  cv.setup();
        cv.getValue();
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Stone Position", cv.getValue());
            telemetry.update();
            stonePosition = cv.getValue();
        }
        waitForStart();

        driveTrain.strafe(10,.4);
        driveTrain.spin(1500,.4);

        switch (stonePosition)
        {
            case(2):
                driveTrain.drive(-10,.5);
                break;
            case(1):
                driveTrain.drive(-7,.5);
                break;
            case(0):
                driveTrain.drive(-3,.5);
                break;
        }

        driveTrain.strafe(-35,.6);
        driveTrain.suckIn();

        driveTrain.drive(12,.4);

        driveTrain.strafe(20, .5);
        driveTrain.spin(100,.5);
        driveTrain.drive(37,.5);
        driveTrain.suckOut();
        driveTrain.drive(-10,.5);
        driveTrain.suckOff();




    }
}