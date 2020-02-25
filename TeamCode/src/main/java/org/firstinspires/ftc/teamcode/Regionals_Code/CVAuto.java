package org.firstinspires.ftc.teamcode.Regionals_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "CV AUTO (RED DEPOT)")
public class CVAuto extends LinearOpMode {

    int stonePosition = -100;
    ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain, true);
        OpenCV cv = new OpenCV(telemetry, hardwareMap);
        imu.imuSetup();
        cv.setupWebCam();
        cv.getValue();
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Stone Position", cv.getValue());
            telemetry.update();
            stonePosition = cv.getValue();
        }

        //positive drive is toward intake
        //positive strafe is toward auto grabber
        waitForStart();

        arm.openRepos();

        arm.lowerAutoArm();

        if(stonePosition == 2)
        {
            driveTrain.drive(8,.55);
        }

        driveTrain.imuStrafe(25,.37, imu);
        if(stonePosition == 0) {
                driveTrain.drive(-6, .55);//here too
        }
        //Grab Skystone
        driveTrain.strafe(2,.3);
        arm.pinchBlock();
        driveTrain.spline(.5,24,315);//stanley and travis are here

        imu.proportionalIMU(0,false);
        //Drive past Line
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(-40, .7, imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(-45, .7, imu);
                break;
            case(2) :
                driveTrain.PropDriveIMU(-50, .7, imu);
                break;
        }
//        //Deposit 1st Skystone
        driveTrain.strafe(2,.4);
        arm.lowerAutoArm();
        arm.raiseAutoArm();
        driveTrain.strafe(-2,.4);

        //Go back for 2nd Stone
        switch(stonePosition)
        {
            case(0):
                driveTrain.PropDriveIMU(76, .75, imu);
                break;
            case(1):
                driveTrain.PropDriveIMU(82, .75, imu);
                break;
            case(2):
                driveTrain.PropDriveIMU(82, .75, imu);
                break;

        }
        if(stonePosition == 0 || stonePosition == 1) {
            //Grab 2nd Stone
            arm.lowerAutoArm();
            driveTrain.imuStrafe(5, .35, imu);
            arm.pinchBlock();
            sleep(100);

            switch(stonePosition) {
                case(0):
                    driveTrain.spline(.6, 21, 315);
                    break;
                case(1):
                    driveTrain.spline(.6, 26, 315);
                    break;
            }
            switch(stonePosition)
            {
                case(0):
                    driveTrain.PropDriveIMU(-88, .75, imu);
                    break;
                case(1):
                    driveTrain.PropDriveIMU(-93, .75, imu);
                    break;

            }

            driveTrain.strafe(8,.5);
            arm.dropBlock();
            sleep(150);
            arm.openRepos();
            imu.proportionalIMU(90,false);
            arm.raiseAutoArm();
            driveTrain.drive(7,.4);
            arm.closeRepos();
            sleep(350);
            driveTrain.drive(-20,.5);
            imu.proportionalIMU(180,true);
            arm.openRepos();
            driveTrain.spline(.6,19,315);
            driveTrain.drive(-25,.6);


        }
        else
        {
            //zuCC the 2nd Stone

        }


//        //Go back for 2nd
//
//        //Grab 2nd Skystone
//        arm.lowerAutoArm();
//        driveTrain.imuStrafe(8,.35,imu);
//        arm.raiseAutoArm();
//        driveTrain.strafe(-9,.4);
//        imu.proportionalIMU(0,false);
//        switch(stonePosition)
//        {
//            case(0):
//                driveTrain.PropDriveIMU(-105, .7, imu);
//                break;
//            case(1):
//                driveTrain.PropDriveIMU(-90, .7, imu);
//                break;
////            case(2) :
////                driveTrain.PropDriveIMU(-93, .6, imu);
////                break;
//        }
//        driveTrain.strafe(6,.4);
//        //Deposit 2nd Skystone
//        arm.dropBlock();
//        sleep(200);
//        arm.raiseAutoArm();
//        arm.openRepos();
//        driveTrain.drive(3,.4);
//        imu.proportionalIMU(90,false);
//        driveTrain.drive(5,.4);
//        arm.closeRepos();
//        sleep(700);
//        driveTrain.drive(-20,.4);
//        imu.proportionalIMU(180, true);




    }
}