package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot12382;

public class DriveTrain extends MotorMethods implements Runnable{

    private final float Ku = .85f;
    private float Kp = Ku/2;

    private DcMotor FL, FR, BL, BR;
    private DcMotor[] driveMotors;

    private final int FRONT_LEFT = 0;
    private final int FRONT_RIGHT = 1;
    private final int BACK_LEFT = 2;
    private final int BACK_RIGHT = 3;

    public InertialMeasurementUnit imu;

    private double previousError;
    private double error;

    private ElapsedTime time;
    double currentTime, pastTime = 0;

    public DriveTrain() {

        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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

        time.reset();
    }

    @Override
    public void run() {}

    public void mecanumDrive(Gamepad gamepad) {

        double drive = gamepad.right_stick_y;
        double strafe = gamepad.right_stick_x;
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
        if (!gamepad.right_bumper) {
            for (int i=0; i < motorPowers.length; i++) {
                motorPowers[i] /= 3;
            }
        }

        if(gamepad.left_trigger > 0)
        {
            FL.setPower(.3);
            FR.setPower(-.3);
            BR.setPower(.3);
            BL.setPower(-.3);
        }
        else if (gamepad.right_trigger > 0)
        {
            FL.setPower(-.3);
            FR.setPower(.3);
            BR.setPower(-.3);
            BL.setPower(.3);
        }
        else {
            FL.setPower(motorPowers[0]);
            FR.setPower(motorPowers[1]);
            BL.setPower(motorPowers[2]);
            BR.setPower(motorPowers[3]);
        }
    }


    public void strafe(strafeDirection direction, double distance, double maxSpeed) {

        int ticks = inchesToTicks(distance);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int directionModifiers[] = direction.modifiers;
        int powerModifiers[] = direction.motorPower.modifiers;

        addTargetPosition(ticks * directionModifiers[FRONT_LEFT], ticks * directionModifiers[FRONT_RIGHT],
                ticks * directionModifiers[BACK_LEFT], ticks * directionModifiers[BACK_RIGHT]);

        setPower(maxSpeed * powerModifiers[FRONT_LEFT], maxSpeed * powerModifiers[FRONT_RIGHT],
                maxSpeed * powerModifiers[BACK_LEFT], maxSpeed * powerModifiers[BACK_RIGHT]);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double currentEncoder = FR.getCurrentPosition();
        double targetEncoder = FR.getTargetPosition();

        while (Math.abs(targetEncoder - currentEncoder) > 30) {

            double errorPID = (targetEncoder - currentEncoder) / targetEncoder;
            currentEncoder = FR.getCurrentPosition();
            double pidModifier = pidController(errorPID, maxSpeed);

            setPower(powerModifiers[FRONT_LEFT] * pidModifier,
                    powerModifiers[FRONT_RIGHT] * pidModifier,
                    powerModifiers[BACK_LEFT] * pidModifier,
                    powerModifiers[BACK_RIGHT] * pidModifier);
        }

        setPower(0);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turn(double target, double maxSpeed) {

        double angle = imu.calculateError(target);
        double pidModifier = pidController(angle, maxSpeed);

        while (Math.abs(angle) > 0.01) {
            int isCC = (int) ( angle / (Math.abs(angle)));
            angle = imu.calculateError(target);
            pidModifier = pidController(angle, maxSpeed);
            setPower(pidModifier * -isCC,
                    pidModifier * isCC,
                    pidModifier * -isCC,
                    pidModifier * isCC);
        }
        setPower(0);
    }


    private double pidController(double distance, double maxSpeed) {

        double pidOutput = 0;
        error =  distance;

        double p;
        double i = 0;
        double d = 0;

        currentTime = time.startTime();

        if (currentTime - pastTime >= 20) {

            p = Math.abs(error) + .15;
            i += Math.abs(error * 0.02);
            d += Math.abs((error - previousError) / 2);
            currentTime = pastTime;
            previousError = error;
            pidOutput = p + i + d;
        }
        if (pidOutput > maxSpeed) {
            pidOutput = maxSpeed;
        }

        return pidOutput;
    }

    private int inchesToTicks(double distance) { return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6)); }

    public enum strafeDirection {
        FORWARD(strafePower.FORWARD,-1,-1,-1,-1),
        BACKWARD(strafePower.BACKWARD,1,1,1,1),
        LEFT(strafePower.LEFT,1,-1,-1,1),
        RIGHT(strafePower.RIGHT,-1,1,1,-1);
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
        LEFT(1,-1,-1,1),
        RIGHT(-1,1,1,-1);
        int[] modifiers;
        strafePower(int... mods) {
            modifiers = mods;
        }
    }
}