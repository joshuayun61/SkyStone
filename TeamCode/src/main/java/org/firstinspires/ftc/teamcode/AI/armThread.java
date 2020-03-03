package org.firstinspires.ftc.teamcode.AI;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Arm;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class armThread extends Thread {

    Arm arm;

    public armThread(Telemetry telemetry, Arm passArm) {

        arm = passArm;
    }


    @Override
    public void run()
    {
        arm.autoDrop();
    }

}
