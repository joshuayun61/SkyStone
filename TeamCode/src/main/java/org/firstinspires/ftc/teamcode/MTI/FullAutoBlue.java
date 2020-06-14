package org.firstinspires.ftc.teamcode.MTI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.armThread;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;
import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.OpenCV;

@Autonomous(name = "FULL BLUE")
public class FullAutoBlue extends LinearOpMode
{
    private enum States
    {
        GRAB_SKYSTONE,
        PLACE_1STSTONE,
        TURN_FOUNDATION,
        GRAB_2NDSKYSTONE,
        PLACE_2NDSTONE,
        FINAL_POSITION,
        END;
    }

    int stonePosition = -100;
    States currentState;
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
        armThread stackTry = new armThread(telemetry,arm,1);
        OpenCV cv = new OpenCV(telemetry, hardwareMap, false);
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
                    arm.tipOut();
                    switch (stonePosition)
                    {
                        case (0):
                            driveTrain.newDrive(-41, stateTime, imu, 14,  false, this);
                            driveTrain.halt();
                            break;
                        case (1):
                            driveTrain.newDrive(-41, stateTime, imu, 2, false,this);
                            driveTrain.halt();
                            break;
                        case (2):
                            driveTrain.newDrive(-40, stateTime, imu, -15, false,this);
                            driveTrain.halt();
                            break;
                    }
                    stateTime.reset();
                    if (stonePosition == 1)
                        //   driveTrain.newDrive(9, stateTime, imu, 0, false,this);
                        driveTrain.driveConst(7,.34,this);
                    else
                        driveTrain.driveConst(6,.4,this);
                    //  driveTrain.newDrive(6, stateTime, imu, 0, false,this);
                    stateTime.reset();
                    lockStone.run();
                    driveTrain.newDrive(30, stateTime, imu, -90, false,this);
                    driveTrain.suckOff();
                    stateTime.reset();
                    if(stonePosition == 2)
                    {
                        driveTrain.newDrive(63, stateTime, imu, -100, false,this);
                    }
                    else
                    {
                        driveTrain.newDrive(58, stateTime, imu, -105, false, this);
                    }
                    nextState(States.PLACE_1STSTONE);
                    break;
                case PLACE_1STSTONE:
                    if (!turn)
                    {
                        if (imu.PISend(180, true) > 0.01)
                        {
                            driveTrain.turn(imu.PISend(180, true), true, stateTime);
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
                        if(stonePosition != 0)
                            driveTrain.driveConst(4,.45,this);
                        else
                            driveTrain.driveConst(4,.46,this);
                        sleep(300);
                        driveTrain.openRepos();
                        sleep(500);
                        if(stonePosition != 0)
                            driveTrain.driveConst(-14,.57,this);
                        else
                            driveTrain.driveConst(-20,.57,this);
                        driveTrain.halt();
                        nextState(States.TURN_FOUNDATION);
                    }
                    break;
                case TURN_FOUNDATION:
                    driveTrain.openRepos();
                    sleep(300);
                    driveTrain.pivotTurn(-20,.63,false, this);
                    driveTrain.pivotTurn(-28,.67,false, this);
                    // driveTrain.pivotTurn(-50,.63,true, this);
                    driveTrain.halt();
                    driveTrain.closeRepos();
                    nextState(States.GRAB_2NDSKYSTONE);
                    moves = 0;
                    break;
                case GRAB_2NDSKYSTONE:
                    switch (stonePosition)
                    {
                        case (0):
                            driveTrain.newDrive(-65, stateTime, imu, -88, false, this);
                            driveTrain.suckIn();
                            driveTrain.halt();
                            stateTime.reset();
                            break;
                        case (1):
                            driveTrain.newDrive(-74, stateTime, imu, -88, false, this);
                            driveTrain.suckIn();
                            driveTrain.halt();
                            stateTime.reset();
                            break;
                        case (2):
                            driveTrain.newDrive(-79, stateTime, imu, -88, false, this);
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
                        driveTrain.newDrive(-11, stateTime, imu, -15, false, this);
                        driveTrain.driveConst(-6, .35, this);
                        driveTrain.driveConst(7, .45, this);
                        lockStone.run();
                        driveTrain.suckOff();
                        driveTrain.newDrive(25, stateTime, imu, -90, false, this);
                    }
                    else
                    {
                        driveTrain.halt();
                        driveTrain.pivotTurn(-15,.45,false, this);
                        driveTrain.suckIn();
                        driveTrain.driveConst(-7,.35,this);
                        driveTrain.driveConst(8,.5,this);
                        lockStone.run();
                        driveTrain.suckOff();
                        driveTrain.newDrive(25, stateTime, imu, -90, false, this);
                    }
                    nextState(States.PLACE_2NDSTONE);

                    break;
                case PLACE_2NDSTONE:
                    driveTrain.halt();
                    stateTime.reset();
                    switch (stonePosition)
                    {
                        case(0):
                            driveTrain.newDrive(35, stateTime,imu,-90,false, this);
                            break;
                        case(1):
                            driveTrain.newDrive(40,stateTime,imu,-90,false,this);
                            break;
                        case(2) :
                            driveTrain.newDrive(44, stateTime,imu,-90,false,this);
                            break;
                    }
                    stackTry.start();
                    stateTime.reset();
                    driveTrain.halt();
                    //driveTrain.newDrive(30,stateTime,imu,90,false,this);
                    driveTrain.driveConst(30,.35,this);
                    driveTrain.halt();
                    sleep(150);
                    stateTime.reset();
                    driveTrain.newDrive(-40,stateTime,imu,-90,false, this);
                    nextState(States.FINAL_POSITION);
                    break;
                case FINAL_POSITION:
                    driveTrain.halt();
                    switch(stonePosition)
                    {
                        case(1):
                            driveTrain.suckIn();
                            stateTime.reset();
                            driveTrain.newDrive(-20,stateTime,imu,15,false,this);
                            driveTrain.pivotTurn(15,.4, true, this);
                            driveTrain.driveConst(7,.7,this);
                            driveTrain.driveConst(-7,.6,this);
                            break;
                        case(2):
                            driveTrain.suckIn();
                            driveTrain.tapeOutSlow();
                            arm.openGrabber();
                            driveTrain.newDrive(-35,stateTime,imu,35, false, this);
                            break;
                    }
                    nextState(States.END);
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
