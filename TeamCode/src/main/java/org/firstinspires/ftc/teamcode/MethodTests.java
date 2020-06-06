package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewRobot.DriveTrainV3;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;
import org.firstinspires.ftc.teamcode.NewRobot.Odometry;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Methods Test")
public class MethodTests extends LinearOpMode {

    private enum States
    {
        FIRST,
        DRIVE,
        DRIVE_AND_TURN,
        TURN;
    }
    States currentState;
    ElapsedTime stateTime = new ElapsedTime();
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        IMU imu = new IMU(telemetry, hardwareMap);
        imu.imuSetup();

        currentState = States.FIRST;

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Ready", true);
            telemetry.update();
        }

        while (opModeIsActive())
        {
            switch (currentState) {
                case FIRST:
                    telemetry.addData("In FIRST State", true);
                    telemetry.update();
                    sleep(2000);
                    nextState(States.TURN);
                    break;
                case TURN:
                    if (imu.PISend(90, false) > 0.001) {
                        driveTrain.turn(imu.PISend(90, false), false, stateTime);
                    } else {
                        driveTrain.halt();
                                             nextState(States.DRIVE);

                    }

                    break;
                case DRIVE:
                    this.stop();
            }

        }
    }

    public void nextState(States next)
    {
        currentState = next;
        stateTime.reset();
    }
}
