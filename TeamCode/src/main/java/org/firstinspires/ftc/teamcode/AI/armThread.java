package org.firstinspires.ftc.teamcode.AI;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class armThread extends Thread {

    private Arm arm;
    private boolean lockBlock;
    private int stack;


    public armThread(Telemetry telemetry, Arm passArm, boolean lock)
    {
        arm = passArm;
        lockBlock = lock;
    }
    public armThread(Telemetry telemetry, Arm passArm)
    {
        arm = passArm;
    }
    public armThread(Telemetry telemetry, Arm passArm, int s)
    {
        stack = 1;
        arm = passArm;
    }

    @Override
    public void run()
    {
        if(stack == 1)
        {
            arm.stackDrop();
        }
        else if(lockBlock)
        {
            arm.autoLock();
        }
        else
            arm.autoDrop();
    }

}
