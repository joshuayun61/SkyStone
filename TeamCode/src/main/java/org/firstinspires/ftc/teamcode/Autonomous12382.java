package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.AI.VufandTensor;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.InetialMUnit;

@Autonomous
public class Autonomous12382 extends LinearOpMode {
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        TensorSense sense = new TensorSense(telemetry, hardwareMap);
        InetialMUnit imu = new InetialMUnit(telemetry, hardwareMap, driveTrain);

        //sense.setupTfod();
        imu.imuSetup();

        waitForStart();

        driveTrain.spin(750, .3);

        //driveTrain.drive(24,.35);



       /* while(opModeIsActive()) {
            sense.runTfod();

            telemetry.addData("Position", sense.stonePosition);
            telemetry.update();
            sleep(2000);
        }

        switch (sense.stonePosition) {
            case (-1) :
                driveTrain.strafe(10, 0.25);
                break;
            case (1) :
                driveTrain.strafe(-10, 0.25);
        }*/
    }
}
