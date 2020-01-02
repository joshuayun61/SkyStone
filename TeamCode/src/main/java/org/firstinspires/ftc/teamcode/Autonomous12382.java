package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.AI.VufandTensor;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.InetialMUnit;
import org.firstinspires.ftc.teamcode.Robot.Arm;

@Autonomous(name = "Autonomous Red Depot")
public class Autonomous12382 extends LinearOpMode {
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        TensorSense sense = new TensorSense(telemetry, hardwareMap);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain);
        sense.setupTfod();
        imu.imuSetup();

        waitForStart();


//        driveTrain.driveAndArm(24,.4,arm.Intake,true);
//        driveTrain.strafe(4,.4);
//        sleep(500);
//
//        while(sense.stonePosition == -100) {
//            arm.openArm();
//            sense.runTfod();
//            telemetry.addData("Position", sense.stonePosition);
//            telemetry.update();
//            sleep(1000);
//        }
//
//        for (int i = 0; i < 100; i++) {
//            sense.runTfod();
//            telemetry.clear();
//            telemetry.addData("Position", sense.stonePosition);
//            telemetry.update();
//        }
//        switch (sense.stonePosition) {
//            case (-1) :
//                driveTrain.strafe(-6, 0.3);
//                break;
//            case (1) :
//                driveTrain.strafe(7, 0.3);
//        }
//
//
//       //grab stone and pull out and turn
//       driveTrain.drive(13,.4);
//       arm.closeArm();
//       driveTrain.drive(-6,.5);
//       imu.proportionalIMU(90);
//
//       //drive depending on where the skystone was
//        switch(sense.stonePosition)
//        {
//            case (-1) :
//                driveTrain.PropDrive(73, .5);
//                break;
//            case (0) :
//                driveTrain.PropDrive(82, .5);
//                break;
//            case (1) :
//                driveTrain.PropDrive(91,.5);
//                break;
//
//        }
//
//
//
//        //raise arm
//        arm.closeArm();
//        arm.raisePH(600);
//
//        //turn and drop
//        imu.proportionalIMU(0, .2,.3);
//        driveTrain.drive(10,.5);
//        arm.Intake.setPosition(.6);
//        sleep(200);

        //pull out
        driveTrain.drive(-10,.6);

        //180 turn and open repos
        imu.proportionalIMU(180);
        driveTrain.reposOpen();


        //move to center of foundation
//        driveTrain.strafe(10,.5);

        //pull foundation
        driveTrain.driveAndArm(-16,.3,arm.Intake,false);
        driveTrain.reposClose(); sleep(300);
//        driveTrain.drive(42,.7);
//
//        sleep(300);
//        //rotate foundation
//        imu.proportionalIMU(-90,.5,.6);
//        driveTrain.strafe(-10,.5);
//        driveTrain.PropDrive(-20,.4);
//        driveTrain.reposOpen();
//        sleep(300);
//        driveTrain.strafe(-8,.5);
//        driveTrain.PropDrive(25, .6);
//        sleep(1000);

        /* arm.ground();

        //drive for next SkyStone
        switch(sense.stonePosition) {
            case (-1):
                driveTrain.driveAndArm(-90, .8,arm.Intake, true);
                break;
            case (0):
                driveTrain.driveAndArm(-98, .8,arm.Intake, true);
                break;
            case (1):
                driveTrain.driveAndArm(-110, .8,arm.Intake, true);
                break;
        }

        driveTrain.spin(-770,.5);
        imu.turnOriginalAngle(0,.3);
        driveTrain.drive(3, .5);
        arm.closeArm();
        driveTrain.drive(-3,.5);*/









    }
}
