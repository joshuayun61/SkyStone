package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot12382;

public class Reposition {

    Servo leftRepos = Robot12382.hardwareMap.get(Servo.class, "leftR");
    Servo rightRepos = Robot12382.hardwareMap.get(Servo.class, "rightR");

    public void reposition(boolean open, boolean close) {
        if (open) {
            leftRepos.setPosition(.4);
            rightRepos.setPosition(.6);
        }
        else if (close) {
            leftRepos.setPosition(.9);
            rightRepos.setPosition(.2);
        }
    }

    private void open() {
        leftRepos.setPosition(.4);
        rightRepos.setPosition(.6);
    }

    private void close() {
        leftRepos.setPosition(.9);
        rightRepos.setPosition(.2);
    }
}