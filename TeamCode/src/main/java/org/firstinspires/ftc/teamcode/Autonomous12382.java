package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.AI.VufandTensor;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.InetialMUnit;
import org.firstinspires.ftc.teamcode.Robot.Arm;

@Autonomous
public class Autonomous12382 extends LinearOpMode {
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1);
        sense.setupTfod();
        imu.imuSetup();

        waitForStart();

        arm.openArm();

        driveTrain.drive(24,.35);
        driveTrain.strafe(5,.35);


       while(sense.stonePosition == -100) {
            sense.runTfod();
            telemetry.addData("Position", sense.stonePosition);
            telemetry.update();
            sleep(2000);
       }

       for (int i = 0; i < 90; i++) {
           sense.runTfod();
       }
       telemetry.clear();
       telemetry.addData("Position" , sense.stonePosition);
       telemetry.update();
       sleep(2000);

        switch (sense.stonePosition) {
            case (-1) :
                driveTrain.strafe(-8, 0.25);
                break;
            case (1) :
                driveTrain.strafe(10, 0.25);
        }

       //grab stone and pull out and turn
       driveTrain.drive(12,.3);
       arm.closeArm();
       driveTrain.drive(-6,.45);
       imu.turnToAngle(90,.2);

       //turn to cross and drop
        switch(sense.stonePosition)
        {
            case (-1) :
                driveTrain.drive(45, .5);
                break;
            case (0) :
                driveTrain.drive(57, .5);
                break;
            case (1) :
                driveTrain.drive(69,.5);
                break;

        }
        driveTrain.strafe(36, .4);
        arm.raisePH();
        driveTrain.drive(20,.5);
        arm.openArm();
        driveTrain.drive(-20,.4);
        driveTrain.strafe(-36,.4);




    }
}
