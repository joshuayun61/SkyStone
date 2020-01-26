package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot12382;

public class Intake extends MotorMethods{

    DcMotor leftIntake, rightIntake;

    public Intake() {
        leftIntake = Robot12382.hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = Robot12382.hardwareMap.get(DcMotor.class, "rightIntake");
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        DcMotor[] intakeMotors = new DcMotor[]{leftIntake, rightIntake};
        initDcMotorArray(intakeMotors);
    }

    public void intake(double intakePower) {
        setPower(intakePower);
    }

}