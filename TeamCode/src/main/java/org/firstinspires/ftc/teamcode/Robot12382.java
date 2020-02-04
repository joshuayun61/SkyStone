package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AI.OpenCVTestBench;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;


public class Robot12382 {

    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    public OpenCVTestBench openCVTestBench;

    public DriveTrain driveTrain;

    Robot12382(Telemetry telemetry, HardwareMap hardwareMap) {

        Robot12382.telemetry = telemetry;
        Robot12382.hardwareMap = hardwareMap;

        driveTrain = new DriveTrain();

        //openCVTestBench = new OpenCVTestBench();
    }
}