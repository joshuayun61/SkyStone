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
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1);
        sense.setupTfod();
        imu.imuSetup();

        waitForStart();

        arm.openArm();

        driveTrain.drive(24,.35);
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
                driveTrain.strafe(-7, 0.25);
                break;
            case (1) :
                driveTrain.strafe(10, 0.25);
        }



       //grab stone and pull out and turn
       driveTrain.drive(13,.3);
       arm.closeArm();
       driveTrain.drive(-5,.45);
       imu.turnOriginalAngle(88,.3);

       //drive depending on where the skystone was
        switch(sense.stonePosition)
        {
            case (-1) :
                driveTrain.drive(44, .5);
                break;
            case (0) :
                driveTrain.drive(53, .5);
                break;
            case (1) :
                driveTrain.drive(66,.5);
                break;

        }



        //position in front of build site and place
        arm.raisePH(200);
        driveTrain.strafe(24, .4);
        driveTrain.drive(13,.5);
        arm.openArm();
        driveTrain.drive(-12,.4);

        //strafe back and lower arm to head for next block
        driveTrain.strafe(-24,.4);
        arm.ground();
        imu.turnOriginalAngle(90,.3);

        //drive back
        driveTrain.drive(-67,.5);
        imu.turnOriginalAngle(0,.3);




    }
}
