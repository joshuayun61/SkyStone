package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationReposition {

    Servo rightRepos, leftRepos;

    private static Telemetry telemetry;
    private static HardwareMap hardwareMap;

    private Gamepad gamepad1;

    public FoundationReposition (Telemetry importTelemetry, HardwareMap importHardwareMap, Gamepad gamepad) {

        telemetry = importTelemetry;
        hardwareMap = importHardwareMap;
        gamepad1 = gamepad;

        leftRepos = hardwareMap.get(Servo.class, "leftR");
        rightRepos = hardwareMap.get(Servo.class, "rightR");
    }

    public void reposition() {

        if (gamepad1.b) { //close the servos
            leftRepos.setPosition(.4);
            rightRepos.setPosition(.6);
        }
        else if (gamepad1.a) { //Open the servos
            leftRepos.setPosition(.9);
            rightRepos.setPosition(.2);
        }
    }
}
