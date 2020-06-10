package org.firstinspires.ftc.teamcode.MTI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        OpenCV cv = new OpenCV(telemetry,hardwareMap, true);
        imu.imuSetup();
        cv.setupWebCam();
        stonePosition = cv.getValue();

        currentState = FullAutoRed.States.GRAB_SKYSTONE;

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
                            driveTrain.newDrive(-30, stateTime, imu, 0, this);
                            driveTrain.halt();
                            break;
                        case (2):
                            driveTrain.newDrive(-30, stateTime, imu, 10, this);
                            driveTrain.halt();
                            break;
                    }

                    //TODO: add code to drive back and secure stone.
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
