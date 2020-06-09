package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewRobot.DriveTrainV3;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;
import org.firstinspires.ftc.teamcode.NewRobot.Odometry;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Methods Test")
public class MethodTests extends LinearOpMode {

    private enum States
    {
        GRAB_SKYSTONE,
        DRIVE,
        DRIVE_AND_TURN,
        TURN;
    }
    int stonePosition = -100;
    States currentState;
    ElapsedTime stateTime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        IMU imu = new IMU(telemetry, hardwareMap);
        OpenCV cv = new OpenCV(telemetry,hardwareMap, true);
        imu.imuSetup();
        cv.setupWebCam();
        stonePosition = cv.getValue();

        currentState = States.GRAB_SKYSTONE;

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Ready", true);
            telemetry.addData("StonePosition", stonePosition);
            telemetry.addData("Ultra", driveTrain.ultrasonic());
            telemetry.addData("optical", driveTrain.getOptical());
            telemetry.update();
            stonePosition = cv.getValue();
        }

        while (opModeIsActive())
        {
            switch (currentState)
            {
                case GRAB_SKYSTONE:
                    telemetry.addData("Driving", null);
                    driveTrain.PropDriveIMU(10, .7, imu, 0);
                    nextState(States.DRIVE);
                    break;
                case TURN:
                    if (imu.PISend(90, false) > 0.001) {
                        telemetry.addData("Low", imu.PISend(90, false));
                        driveTrain.turn(imu.PISend(90, false), false, stateTime);
                    } else {
                        driveTrain.halt();
                        nextState(States.DRIVE);
                    }

                    break;
                case DRIVE:
                    this.stop();
            }
            telemetry.update();
        }
    }

    public void nextState(States next)
    {
        currentState = next;
        stateTime.reset();
    }
}
