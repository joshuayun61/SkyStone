package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AI.OpenCVTestBench;


public class Robot12382 {

    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    Robot12382(Telemetry telemetry, HardwareMap hardwareMap) {

        Robot12382.telemetry = telemetry;
        Robot12382.hardwareMap = hardwareMap;

        OpenCVTestBench openCVTestBench = new OpenCVTestBench();
    }
}