package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot12382;

public class Arm extends MotorMethods {

    private DcMotor Slide;
    private Servo Intake;
    private final int buildPlateHeight = 700;
    private final int blockHeight = 1000;

    public Arm() {
        Slide = Robot12382.hardwareMap.get(DcMotor.class, "Slide");
        Intake = Robot12382.hardwareMap.get(Servo.class, "Intake");
        initDcMotorArray(Slide);
    }

    private int slidePosition() { return Slide.getCurrentPosition(); }

    private void setToGroundPosition() {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setTargetPosition(5);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(.9);
        while(Slide.getCurrentPosition() > 25) {
            Robot12382.telemetry.addData("Slide Tick",slidePosition());
            Robot12382.telemetry.update();
        }
    }

    private void moveOneBlockHeight(boolean raise) {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (raise) {
            addTargetPosition(blockHeight);
            setPower(0.9);
        }
        else {
            addTargetPosition(blockHeight);
            setPower(-0.9);
        }
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(isBusy()) {}
    }

    private void movePlatformHeight() {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setTargetPosition(buildPlateHeight);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(slidePosition() > blockHeight + 10 || slidePosition() < blockHeight - 10) {
            Robot12382.telemetry.addData("Slide Tick", slidePosition());
            Robot12382.telemetry.update();
        }
    }

    private void liftBlock() {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setTargetPosition(50);
        setPower(0.9);
        while(slidePosition() < 40) {}
    }

    public void openArm() { Intake.setPosition(1); }
    public void closeArm() { Intake.setPosition(.1); }



    public void moveArm(double left_stick_y, boolean a, boolean b, boolean x,
                        boolean left_bumper, boolean right_bumper, boolean dpad_up,
                        boolean dpad_down) {
        if (a) {
            movePlatformHeight();
        }
        if (b) {
            liftBlock();
        }
        if (x) {
            setToGroundPosition();
        }
        if (dpad_up) {
            moveOneBlockHeight(true);
        }
        if (dpad_down) {
            moveOneBlockHeight(false);
        }
        if (right_bumper) {
            openArm();
        }
        if (left_bumper) {
            closeArm();
        }
        if(left_stick_y != 0)
        {
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setPower(-left_stick_y/1.5);
        }
        if (slidePosition() > 4100) {
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setTargetPosition(4000);
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            setPower(0.8);
            while (isBusy()) {}
        }
    }


}