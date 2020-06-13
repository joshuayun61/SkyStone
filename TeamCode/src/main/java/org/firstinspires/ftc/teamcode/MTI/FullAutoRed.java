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

import java.util.concurrent.TimeUnit;

@Autonomous(name = "FULL RED")
public class FullAutoRed extends LinearOpMode
{
    private enum States
    {
        GRAB_SKYSTONE,
        PLACE_1STSTONE,
        TURN_FOUNDATION,
        GRAB_2NDSKYSTONE,
        PLACE_2NDSTONE,
        FINAL_POSITION,
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
                    driveTrain.openRepos();
                    arm.tipOut();
                    switch (stonePosition)
                    {
                        case (0):
                            driveTrain.newDrive(-41, stateTime, imu, -14,  false, this);
                            driveTrain.halt();
                            break;
                        case (1):
                            driveTrain.newDrive(-41, stateTime, imu, 2, false,this);
                            driveTrain.halt();
                            break;
                        case (2):
                            driveTrain.newDrive(-41, stateTime, imu, 15, false,this);
                            driveTrain.halt();
                            break;
                    }
                    stateTime.reset();
                    if (stonePosition == 1)
                     //   driveTrain.newDrive(9, stateTime, imu, 0, false,this);
                        driveTrain.driveConst(7,.34,this);
                    else if(stonePosition == 2)
                        driveTrain.driveConst(6,.4,this);
                    else
                        driveTrain.driveConst(4,.4,this);
                      //  driveTrain.newDrive(6, stateTime, imu, 0, false,this);
                    stateTime.reset();
                    lockStone.run();
                    driveTrain.newDrive(30, stateTime, imu, 90, false,this);
                    driveTrain.suckOff();
                    stateTime.reset();
                    if(stonePosition == 2)
                    {
                        driveTrain.newDrive(63, stateTime, imu, 100, false,this);
                    }
                    else
                    {
                        driveTrain.newDrive(63, stateTime, imu, 105, false, this);
                    }
                        nextState(States.PLACE_1STSTONE);
                    break;
                case PLACE_1STSTONE:
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
                        if(stonePosition == 2)
                            driveTrain.driveConst(9,.45,this);
                        else
                            driveTrain.driveConst(5,.45,this);
                        driveTrain.closeRepos();
                        sleep(400);
                        driveTrain.driveConst(-14,.57,this);
                        driveTrain.halt();
                        nextState(States.TURN_FOUNDATION);
                    }
                    break;
                case TURN_FOUNDATION:
                    driveTrain.closeRepos();
                    sleep(300);
                    driveTrain.pivotTurn(-20,.75,true, this);
                    if(stonePosition == 1)
                       driveTrain.pivotTurn(-34,.7,true,this);
                        //driveTrain.newDriveFast(-30,stateTime,imu,100,false,this);
                    else
                        driveTrain.pivotTurn(-33,.7,true, this);
                   // driveTrain.pivotTurn(-50,.63,true, this);
                    driveTrain.halt();
                    driveTrain.openRepos();
                    nextState(States.GRAB_2NDSKYSTONE);
                    moves = 0;
                    break;
                case GRAB_2NDSKYSTONE:
                    switch (stonePosition)
                    {
                        case (0):
                            driveTrain.newDrive(-65, stateTime, imu, 80, false, this);
                            driveTrain.suckIn();
                            driveTrain.halt();
                            stateTime.reset();
                            break;
                        case (1):
                            driveTrain.newDrive(-72, stateTime, imu, 75, false, this);
                            driveTrain.suckIn();
                            driveTrain.halt();
                            stateTime.reset();
                            break;
                        case (2):
                            driveTrain.newDrive(-76, stateTime, imu, 75, false, this);
                            //driveTrain.suckIn();
                            driveTrain.halt();
                            stateTime.reset();
                    }
                    if(driveTrain.ultrasonic() > 90)
                    {
                        driveTrain.driveConst(-4, .5, this);
                    }
                    arm.openGrabber();
                    stateTime.reset();
                    if(stonePosition != 2)
                    {
                        driveTrain.newDrive(-14, stateTime, imu, 15, false, this);
                        driveTrain.driveConst(-6, .35, this);
                        driveTrain.driveConst(7, .45, this);
                        lockStone.run();
                        driveTrain.suckOff();
                        driveTrain.newDrive(25, stateTime, imu, 90, false, this);
                    }
                    else
                    {
                        driveTrain.halt();
                        if(stonePosition == 1)
                            driveTrain.pivotTurn(-12,.45,true, this);
                        else if(stonePosition == 2)
                            driveTrain.pivotTurn(-13,.45,true,this);
                        else
                            driveTrain.pivotTurn(-14,.45,true, this);
                        driveTrain.suckIn();
                        if(stonePosition == 1)
                            driveTrain.driveConst(-13,.37,this);
                        else if(stonePosition == 2)
                            driveTrain.driveConst(-10,.37,this);
                        else
                            driveTrain.driveConst(-8,.35,this);
                        driveTrain.driveConst(8,.5,this);
                        lockStone.run();
                        driveTrain.suckOff();
                        driveTrain.newDrive(25, stateTime, imu, 90, false, this);
                    }
                    nextState(States.PLACE_2NDSTONE);

                    break;
                case PLACE_2NDSTONE:
                        driveTrain.halt();
                        stateTime.reset();
                        switch (stonePosition)
                        {
                            case(0):
                                driveTrain.newDrive(38, stateTime,imu,90,false, this);
                                break;
                            case(1):
                                driveTrain.newDrive(40,stateTime,imu,90,false,this);
                                break;
                            case(2) :
                                driveTrain.newDrive(44, stateTime,imu,90,false,this);
                                break;
                        }
                        dropStone.start();
                        stateTime.reset();
                        driveTrain.halt();
                        //driveTrain.newDrive(30,stateTime,imu,90,false,this);
                        driveTrain.driveConst(35,.35,this);
                        driveTrain.halt();
                        sleep(150);
                        stateTime.reset();
                        driveTrain.newDrive(-40,stateTime,imu,90,false, this);
                        nextState(States.FINAL_POSITION);
                    break;
                case FINAL_POSITION:
                    driveTrain.halt();
                    switch(stonePosition)
                    {
                        case(1):
                            driveTrain.suckIn();
                            driveTrain.tapeOutSlow();
                            arm.openGrabber();
                            driveTrain.newDrive(-40,stateTime,imu,27, false, this);
                            break;
                        case(2):
                            driveTrain.suckIn();
                            driveTrain.tapeOutSlow();
                            arm.openGrabber();
                            driveTrain.newDrive(-40,stateTime,imu,27, false, this);
                            break;
                    }
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
                    nextState(States.PLACE_1STSTONE);
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
