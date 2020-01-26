package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Reposition;

public class Robot12382 {

    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    DriveTrain driveTrain;
    Reposition reposition;

    Robot12382(Telemetry telemetry, HardwareMap hardwareMap) {

        Robot12382.telemetry = telemetry;
        Robot12382.hardwareMap = hardwareMap;

       // driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1);
        reposition = new Reposition();
    }



}