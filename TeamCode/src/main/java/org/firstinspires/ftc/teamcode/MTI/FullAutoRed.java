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
        DRIVE,
        TEST,
        DRIVE_AND_TURN,
        TURN;
    }
    int stonePosition = -100;
    FullAutoRed.States currentState;
    ElapsedTime stateTime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1,true);
        Arm arm = new Arm(telemetry,hardwareMap, gamepad2,driveTrain,true);
        IMU imu = new IMU(telemetry, hardwareMap);
        armThread lockStone = new armThread(telemetry, arm, true);
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
                    stateTime.reset();
                    driveTrain.suckIn();
                    arm.openGrabber();
                    switch(stonePosition)
                    {
                        case (0):
                            driveTrain.newDrive(-35, stateTime, imu, -10, this);
                            driveTrain.halt();
                            break;
                        case (1):
                            driveTrain.newDrive(-40, stateTime, imu, 5, this);
                            driveTrain.halt();
                            break;
                        case (2):
                            driveTrain.newDrive(-37, stateTime, imu, 20, this);
                            driveTrain.halt();
                            break;
                    }
                    stateTime.reset();
                    if(stonePosition == 1)
                        driveTrain.newDrive(10, stateTime, imu, 0, this);
                    else
                        driveTrain.newDrive(5, stateTime, imu, 0, this);
                    stateTime.reset();
                    lockStone.run();
                    driveTrain.newDrive(30, stateTime, imu, 90, this);
                    stateTime.reset();
                    driveTrain.newDrive(50, stateTime, imu, 90, this);
                    nextState(States.DRIVE);
                    break;
                case TURN:
                    stateTime.reset();
                    if (imu.PISend(90, false) > 0.001) {
                        telemetry.addData("Low", imu.PISend(90, false));
                        driveTrain.turn(imu.PISend(90, false), false, stateTime);
                    } else {
                        driveTrain.halt();
                        nextState(States.DRIVE);
                    }

                    break;
                case TEST:
                    driveTrain.suckIn();
                    sleep(1500);
                    driveTrain.suckOff();
                    arm.tipInward();
                    sleep(300);
                    arm.closeGrabber();
                    sleep(1000);
                    nextState(States.DRIVE);
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
