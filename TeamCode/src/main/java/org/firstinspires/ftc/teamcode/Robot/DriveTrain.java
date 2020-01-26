package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot12382;

public class DriveTrain extends MotorMethods {

    private final float Ku = .85f;
    private float Kp = Ku/2;

    private DcMotor FL, FR, BL, BR;
    private DcMotor[] driveMotors;

    private final int FRONT_LEFT = 0;
    private final int FRONT_RIGHT = 1;
    private final int BACK_LEFT = 2;
    private final int BACK_RIGHT = 3;

    private InertialMeasurementUnit imu;

    public DriveTrain() {
        FL = Robot12382.hardwareMap.get(DcMotor.class, "FL");
        FR = Robot12382.hardwareMap.get(DcMotor.class, "FR");
        BL = Robot12382.hardwareMap.get(DcMotor.class, "BL");
        BR = Robot12382.hardwareMap.get(DcMotor.class, "BR");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        driveMotors = new DcMotor[]{FL, FR, BL, BR};

        initDcMotorArray(driveMotors);

        imu = new InertialMeasurementUnit();
        imu.imuSetup();
    }

    public void mecanumDrive(Gamepad gamepad) {

        double drive = gamepad.right_stick_y;
        double strafe = -gamepad.right_stick_x;
        double spin = gamepad.left_stick_x;

        double flPower = drive - strafe - spin;
        double frPower = drive + strafe + spin;
        double blPower = drive + strafe - spin;
        double brPower = drive - strafe + spin;

        double[] motorPowers = {flPower, frPower, blPower, brPower};

        if (gamepad.left_bumper) {
            for (int i=0; i < motorPowers.length; i++) {
                motorPowers[i] /= 6;
            }
        }
        else if (!gamepad.right_bumper) {
            for (int i=0; i < motorPowers.length; i++) {
                motorPowers[i] /= 3;
            }
        }

        if(gamepad.right_trigger == 0 && gamepad.left_trigger == 0) {
            FL.setPower(flPower);
            FR.setPower(frPower);
            BL.setPower(blPower);
            BR.setPower(brPower);
        }
        else if(gamepad.right_trigger > 0)
        {
            FL.setPower(.3);
            FR.setPower(-.3);
            BR.setPower(.3);
            BL.setPower(-.3);
        }
        else
        {
            FL.setPower(-.3);
            FR.setPower(.3);
            BR.setPower(-.3);
            BL.setPower(.3);
        }
    }


    public void strafe(strafeDirection direction, double distance, double motorPower) {
        int ticks = inchesToTicks(distance);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int directionModifiers[] = direction.modifiers;
        int powerModifiers[] = direction.motorPower.modifiers;

        addTargetPosition(ticks * directionModifiers[FRONT_LEFT], ticks * directionModifiers[FRONT_RIGHT],
                ticks * directionModifiers[BACK_LEFT], ticks * directionModifiers[BACK_RIGHT]);

        setPower(motorPower * powerModifiers[FRONT_LEFT], motorPower * powerModifiers[FRONT_RIGHT],
                motorPower * powerModifiers[BACK_LEFT], motorPower * powerModifiers[BACK_RIGHT]);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            Robot12382.telemetry.addData("EncoderPosition", FL.getCurrentPosition());
            Robot12382.telemetry.addData("EncoderTarget", ticks);
            Robot12382.telemetry.update();
        }

        setPower(0);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double pidController(double target) {
        double pidOutput = imu.calculateError(target);
        return pidOutput;
    }

    private int inchesToTicks(double distance) { return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6)); }

    public enum strafeDirection {
        FORWARD(strafePower.FORWARD,-1,-1,-1,-1),
        BACKWARD(strafePower.BACKWARD,1,1,1,1),
        LEFT(strafePower.LEFT,1,-1,1,-1),
        RIGHT(strafePower.RIGHT,-1,1,-1,1);
        strafePower motorPower;
        int[] modifiers;
        strafeDirection(strafePower power, int... mods) {
            motorPower = power;
            modifiers = mods;
        }
    }

    private enum strafePower {
        FORWARD(-1,1,-1,1),
        BACKWARD(1,-1,1,-1),
        LEFT(1,-1,1,-1),
        RIGHT(-1,1,-1,1);
        int[] modifiers;
        strafePower(int... mods) {
            modifiers = mods;
        }
    }
}