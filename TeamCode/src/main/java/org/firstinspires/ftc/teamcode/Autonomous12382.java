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
        driveTrain.strafe(2,.4);

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
                driveTrain.strafe(-7, 0.4);
                break;
            case (1) :
                driveTrain.strafe(8, 0.4);
        }


     //  grab stone and pull out and turn
       driveTrain.drive(14,.4);
       arm.closeArm();
       sleep(500);
       driveTrain.drive(-10,.5);
       imu.proportionalIMU(90, false);
       driveTrain.reposOpen();

       //drive depending on where the skystone was
        switch(sense.stonePosition)
        {
            case (-1) :
                driveTrain.PropDrive(69, .3);
                break;
            case (0) :
                driveTrain.PropDrive(78, .3);
                break;
            case (1) :
                driveTrain.PropDrive(85,.3);
                break;

        }

        // TODO: 1/8/2020 Change this into light mode because it is impossible to read
        // TODO: 1/8/2020 Servos are great


        //raise arm
        arm.Intake.setPosition(.1);
        arm.raisePH(600);

        //turn and drop
        driveTrain.spin(-780, .3);
        driveTrain.drive(15,.5);
        arm.Intake.setPosition(.6);
        sleep(200);
        driveTrain.drive(2,.6);
        driveTrain.reposClose();
        sleep(500);

        driveTrain.drive(-23,.5);

        imu.proportionalIMU(90, true);

        driveTrain.drive(9,.6);
        switch(sense.stonePosition){
            case(-1) :
                driveTrain.strafe(3, .4);
                break;
            case(0) :
                driveTrain.strafe(2,.4);
                break;
            case(1) :
                driveTrain.strafe(5, .4);
        }
        driveTrain.reposOpen();
        sleep(500);
        driveTrain.drive(-6,.6);
        arm.ground();
        driveTrain.drive(-33,.5);


        //pull foundation










    }
}
