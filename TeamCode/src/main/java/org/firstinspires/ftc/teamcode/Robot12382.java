package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AI.OpenCVTestBench;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;


public class Robot12382 {

    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    public OpenCVTestBench openCVTestBench;
    public DriveTrain driveTrainTeleOp;
    public DriveTrain driveTrainStrafe;
    public DriveTrain driveTrainTurn;

    Robot12382(Telemetry telemetry, HardwareMap hardwareMap, final Gamepad gamepad1, final Gamepad gamepad2) {

        Robot12382.telemetry = telemetry;
        Robot12382.hardwareMap = hardwareMap;

        driveTrainTeleOp = new DriveTrain() {
            @Override
            public void run() {
                mecanumDrive(gamepad1);
            }
        };
    }

    Robot12382(Telemetry telemetry, HardwareMap hardwareMap) {

    }
}