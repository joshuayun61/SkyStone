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

          DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        // TensorSense sense = new TensorSense(telemetry, hardwareMap);
          IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
           Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        // sense.setupTfod();
        OpenCV cv = new OpenCV(telemetry, hardwareMap);
        imu.imuSetup();
        cv.setup();
        cv.getValue();
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Stone Position", cv.getValue());
            telemetry.update();
            stonePosition = cv.getValue();
        }
        waitForStart();

        switch (stonePosition)
        {
            case(2):
                driveTrain.drive(3,.7);
                break;
            case(1):
                driveTrain.drive(-4,.5);
                break;
            case(0):
                driveTrain.drive(-12,5);
                break;
        }

        driveTrain.strafe(45,.6);
        driveTrain.suckIn();

        driveTrain.drive(12,.4);

        driveTrain.strafe(-19, .5);
        driveTrain.spin(-75,.4);
        switch(stonePosition)
        {
            case(2):
                driveTrain.drive(45,.5);
                break;
            case(1):
                driveTrain.drive(55,.5);
                break;
            case(0):
                driveTrain.drive(60,.5);
                break;

        }
        driveTrain.suckOut();
        driveTrain.drive(-13,.3);
        driveTrain.suckOff();

//        driveTrain.suckOff();
//        driveTrain.drive(25,.5);
//        imu.proportionalIMU(-90, true);
//        arm.openRepos();
//        driveTrain.drive(-17,.3);
//        arm.closeRepos();
//        sleep(500);
//        imu.proportionalIMU(180,false);




    }
}