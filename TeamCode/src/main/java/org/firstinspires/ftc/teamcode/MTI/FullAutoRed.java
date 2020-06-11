package org.firstinspires.ftc.teamcode.MTI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.armThread;
import org.firstinspires.ftc.teamcode.MethodTests;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "FULL RED")
public class FullAutoRed extends LinearOpMode
{
    private enum States
    {
        GRAB_SKYSTONE,
        PLACE_1stSTONE,
        GRAB_2NDSKYSTONE,
        DRIVE,
        TEST,
        DRIVE_AND_TURN,
        TURN,
        END;
    }

    int stonePosition = -100;
    FullAutoRed.States currentState;
    ElapsedTime stateTime = new ElapsedTime();
    private int moves = 0;
    private boolean turn = false;

    public void runOpMode() throws InterruptedException
    {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1, true);
        Arm arm = new Arm(telemetry, hardwareMap, gamepad2, driveTrain, true);
        IMU imu = new IMU(telemetry, hardwareMap);
        armThread lockStone = new armThread(telemetry, arm, true);
        armThread dropStone = new armThread(telemetry, arm, false);
        OpenCV cv = new OpenCV(telemetry, hardwareMap, true);
        imu.imuSetup();
        cv.setupWebCam();
        stonePosition = cv.getValue();

        currentState = States.GRAB_SKYSTONE;

        while (!isStopRequested() && !opModeIsActive())
        {
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
                    stateTime.reset();
                    driveTrain.suckIn();
                    arm.openGrabber();
                    switch (stonePosition)
                    {
                        case (0):
                            driveTrain.newDrive(-37, stateTime, imu, -10,  false, this);
                            driveTrain.halt();
                            break;
                        case (1):
                            driveTrain.newDrive(-40, stateTime, imu, 5, false,this);
                            driveTrain.halt();
                            break;
                        case (2):
                            driveTrain.newDrive(-37, stateTime, imu, 20, false,this);
                            driveTrain.halt();
                            break;
                    }
                    stateTime.reset();
                    if (stonePosition == 1)
                        driveTrain.newDrive(9, stateTime, imu, 0, false,this);
                    else
                        driveTrain.newDrive(6, stateTime, imu, 0, false,this);
                    stateTime.reset();
                    lockStone.run();
                    driveTrain.newDrive(30, stateTime, imu, 90, false,this);
                    driveTrain.suckOff();
                    stateTime.reset();
                    driveTrain.newDrive(55, stateTime, imu, 100, false,this);
                    nextState(States.PLACE_1stSTONE);
                    break;
                case PLACE_1stSTONE:
                    if (!turn)
                    {
                        if (imu.PISend(180, false) > 0.01)
                        {
                            driveTrain.turn(imu.PISend(180, false), false, stateTime);
                        }
                        else
                        {
                            driveTrain.halt();
                            turn = true;
                        }
                    }
                    else
                    {
                        lockStone.interrupt();
                        dropStone.start();
                        stateTime.reset();
                        driveTrain.driveConst(5,.45,this);
                        driveTrain.closeRepos();
                        sleep(300);
                        driveTrain.driveConst(-16,.57,this);
                        driveTrain.newDrive(-20, stateTime, imu, 90, true, this);
                        driveTrain.openRepos();
                        nextState(States.GRAB_2NDSKYSTONE);
                    }
                    break;
                case GRAB_2NDSKYSTONE:
                    driveTrain.newDrive(-40, stateTime, imu, 90,  false, this);
                    nextState(States.END);
                    break;
                case TURN:
                    stateTime.reset();
                    if (imu.PISend(90, false) > 0.001)
                    {
                        telemetry.addData("Low", imu.PISend(90, false));
                        driveTrain.turn(imu.PISend(90, false), false, stateTime);
                    } else
                    {
                        driveTrain.halt();
                        nextState(States.DRIVE);
                    }

                    break;
                case TEST:
                    nextState(States.PLACE_1stSTONE);
                    break;
                case END:
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
