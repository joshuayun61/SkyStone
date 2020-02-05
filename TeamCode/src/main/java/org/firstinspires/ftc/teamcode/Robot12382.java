package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot12382 {

    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    public DriveTrain driveTrain;

    /**
     * Autonomous version of the Robot12382 class
     * overrides run with the move (Autonomous) method
     * @param telemetry Telemetry for the other classes to use
     * @param hardwareMap HardwareMap for the other classes to use
     */
    Robot12382(Telemetry telemetry, HardwareMap hardwareMap) {

        Robot12382.telemetry = telemetry;
        Robot12382.hardwareMap = hardwareMap;

        driveTrain = new DriveTrain() {
            @Override
            public void run() {
                move();
            }
        };

    }

    /**
     * TeleOp version of the Robot12382 Class
     * overrides run with the mecanumDrive (TeleOp) method
     * and sets the gamepad for the TeleOp
     * @param telemetry Telemetry for the other classes to use
     * @param hardwareMap HardwareMap for the other classes to use
     * @param gamepad Which Gamepad the TeleOp will use for the drive (gamepad 1)
     */
    Robot12382(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad){

        Robot12382.telemetry = telemetry;
        Robot12382.hardwareMap = hardwareMap;

        driveTrain = new DriveTrain() {
            @Override
            public void run() {
                mecanumDrive();
            }
        };

        driveTrain.setGamepad(gamepad);
    }
}