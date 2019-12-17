package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.AI.VufandTensor;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;

@Autonomous
public class Autonomous12382 extends LinearOpMode {
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap);

        sense.setupTfod();

        imu.imuSetup();
        waitForStart();


        driveTrain.drive(16,.25);

        while(sense.stonePosition == -100) {
            sense.runTfod();
            sleep(2000);
        }

        telemetry.addData("Position", sense.stonePosition);
        telemetry.update();
        sleep(2000);

        switch (sense.stonePosition) {
            case (-1) :
                driveTrain.strafe(10, 0.25);
                break;
            case (1) :
                driveTrain.strafe(-10, 0.25);
        }
    }
}
