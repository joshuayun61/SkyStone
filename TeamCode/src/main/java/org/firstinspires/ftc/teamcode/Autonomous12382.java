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


        driveTrain.driveAndArm(24,.35, arm.Intake,true);
        driveTrain.strafe(5,.4);


       while(sense.stonePosition == -100) {
            sense.runTfod();
            telemetry.addData("Position", sense.stonePosition);
            telemetry.update();
            sleep(1000);
       }

       for (int i = 0; i < 25; i++) {
           sense.runTfod();
           telemetry.clear();
           telemetry.addData("Position", sense.stonePosition);
           telemetry.update();
       }


        switch (sense.stonePosition) {
            case (-1) :
                driveTrain.strafe(-7, 0.3);
                break;
            case (1) :
                driveTrain.strafe(7, 0.3);
        }



       //grab stone and pull out and turn
       driveTrain.drive(13,.4);
       arm.closeArm();
       driveTrain.drive(-6,.5);
       imu.turnOriginalAngle(88,.3);

       //drive depending on where the skystone was
        switch(sense.stonePosition)
        {
            case (-1) :
                driveTrain.drive(68, .6);
                break;
            case (0) :
                driveTrain.drive(77, .6);
                break;
            case (1) :
                driveTrain.drive(86,.6);
                break;

        }



        //raise arm
        arm.closeArm();
        arm.raisePH(600);
        //turn and drop
        driveTrain.spin(-770,.7);
        driveTrain.drive(10,.5);
        arm.Intake.setPosition(.6);
        //pull out
        driveTrain.drive(-10,.7);
        //180 turn and open repos
        driveTrain.spin(1550, .5);
        driveTrain.reposOpen();
        //move to center of foundation
        driveTrain.strafe(14,.5);
        //pull foundation
        driveTrain.drive(-12,.4);
        driveTrain.reposClose();
        driveTrain.drive(35,.6);
        //rotate foundation
        imu.turnOriginalAngle(-90,.6);
        sleep(1000);

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
