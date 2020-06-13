package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AI.armThread;
import org.firstinspires.ftc.teamcode.NewRobot.DriveTrainV3;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;
import org.firstinspires.ftc.teamcode.NewRobot.Odometry;
import org.firstinspires.ftc.teamcode.Robot.Arm;
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
    private int moves = 0;
    States currentState;
    ElapsedTime stateTime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad2, driveTrain,true);
        IMU imu = new IMU(telemetry, hardwareMap);
        OpenCV cv = new OpenCV(telemetry,hardwareMap, false);
        armThread dropStone = new armThread(telemetry, arm, false);
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
                    if(moves == 0)
                    {
                        stateTime.reset();
                        //dropStone.start();
                        sleep(5000);
                        moves++;
                    }
                    else
                    {
                        driveTrain.halt();
                        nextState(States.DRIVE);
                    }
                    break;
                case TURN:
                    driveTrain.pivotTurn(-30,.4,true, this);
                    nextState(States.DRIVE);
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
