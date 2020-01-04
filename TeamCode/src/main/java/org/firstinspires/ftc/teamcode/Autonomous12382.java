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


        driveTrain.driveAndArm(24,.4,arm.Intake,true);
        driveTrain.strafe(4,.4);
        sleep(500);

        while(sense.stonePosition == -100) {
            arm.openArm();
            sense.runTfod();
            telemetry.addData("Position", sense.stonePosition);
            telemetry.update();
            sleep(1000);
        }

        for (int i = 0; i < 100; i++) {
            sense.runTfod();
            telemetry.clear();
            telemetry.addData("Position", sense.stonePosition);
            telemetry.update();
        }
        switch (sense.stonePosition) {
            case (-1) :
                driveTrain.strafe(-6, 0.3);
                break;
            case (1) :
                driveTrain.strafe(7, 0.3);
        }


       //grab stone and pull out and turn
       driveTrain.drive(13,.4);
       arm.closeArm();
       driveTrain.drive(-6,.5);
       imu.proportionalIMU(90);

       //drive depending on where the skystone was
        switch(sense.stonePosition)
        {
            case (-1) :
                driveTrain.PropDrive(73, .5);
                break;
            case (0) :
                driveTrain.PropDrive(82, .5);
                break;
            case (1) :
                driveTrain.PropDrive(91,.5);
                break;

        }



        //raise arm
        arm.closeArm();
        arm.raisePH(600);

        //turn and drop
        imu.proportionalIMU(0, .2,.3);
        driveTrain.drive(10,.5);
        arm.Intake.setPosition(.6);
        sleep(200);


//        driveTrain.reposClose();
//        sleep(500);
//
//        driveTrain.PropDrive(-30,.6);
//
//        imu.proportionalIMU(90);



        //pull foundation










    }
}
