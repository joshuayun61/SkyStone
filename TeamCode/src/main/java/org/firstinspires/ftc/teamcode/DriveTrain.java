package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain extends MotorMethods implements Runnable{

    private final float Ku = .85f;
    private float Kp = Ku/2;

    private DcMotor FL, FR, BL, BR;
    private DcMotor[] driveMotors;

    private final int FRONT_LEFT = 0;
    private final int FRONT_RIGHT = 1;
    private final int BACK_LEFT = 2;
    private final int BACK_RIGHT = 3;


    private strafeDirection direction;
    private double target;
    private double maxSpeed;

    private Gamepad gamepad;

    public InertialMeasurementUnit imu;

    private ElapsedTime time;
    private double currentTime = 0;
    private double pastTime = 0;

    private double previousError;

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

    /**
     *Gets overridden when the driveTrain is instantiated with Anonymous Subclass, used for threading
     */
    @Override
    public void run() {}

    /**
     * Sets the gamepad for the driveTrain thread to use during TeleOp
     * @param gamepad Gamepad to be used in the TeleOp driveTrain, usually gamepad1
     */
    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * TeleOp method that runs in a thread in the TeleOp Class
     */
    public void mecanumDrive() {

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


    /**
     * Sets the parameters for the move method which should change after every autonomous command to change how far, in which
     * direction, and how fast the robot should be moving
     * @param direction The direction Turn, Spline, Strafe, Move Forward/Backward the Robot is taking.
     * @param target The destination (Encoder tick, IMU radian, etc.)
     * @param maxSpeed The max speed the robot can go after PID controller is accounted for.
     */
    public void setMoveParameters(strafeDirection direction, double target, double maxSpeed) {
        this.direction = direction;
        this.target = target;
        this.maxSpeed = maxSpeed;

    }

    /**
     * The command in a thread that takes the parameters of the setMoveParameters and actually move the
     * Robot based on those parameters
     */
    public void move() {

        if (direction.equals(strafeDirection.CCW) || direction.equals(strafeDirection.CW)) {
            runIMUMotors(target, maxSpeed);
        }
        else if (direction.equals(strafeDirection.FORWARD) || direction.equals(strafeDirection.BACKWARD)){
            int ticks = inchesToTicksDrive(target);
            runEncoderMotors(ticks, direction, maxSpeed);
        }
        else if (direction.equals(strafeDirection.LEFT) || direction.equals(strafeDirection.RIGHT)) {
            int ticks = inchesToTicksStrafe(target);
            runEncoderMotors(ticks, direction, maxSpeed);
        }
        else {
            int ticks = inchesToTicksSpline(target);
            runEncoderMotors(ticks, direction, maxSpeed);
        }


    }

    private void runEncoderMotors(int ticks, strafeDirection direction, double maxSpeed) {

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    private void runIMUMotors(double target, double maxSpeed) {
        double angle = imu.calculateError(target);
        double pidModifier;

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


    /**
     *
     * @param error Current error that is passed into the PID controller
     * @param maxSpeed The cap that the PID can return to prevent it from pusing the robot too fast
     * @return The Speed that the Robot should go
     */
    private double pidController(double error, double maxSpeed) {

        double pidOutput = 0;

        double p;
        double i = 0;
        double d = 0;

        currentTime = time.startTime();

        if (currentTime - pastTime >= 20) {

            p = Kp * Math.abs(error) + .15;
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

    /**
     * Simple conversions based on the SPLINE, STRAFE, DRIVE measurments for the different move functions
     * @param distance the distance in inches, that you want to move
     * @return
     */
    private int inchesToTicksDrive(double distance) { return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6)); }
    private int inchesToTicksStrafe(double distance) { return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6)); }
    private int inchesToTicksSpline(double distance) { return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6)); }

    /**
     * The enums that are responsible for making sure that the motors are negated correctly for the encoders
     */
    public enum strafeDirection {
        FORWARD(strafePower.FORWARD,-1,-1,-1,-1),
        BACKWARD(strafePower.BACKWARD,1,1,1,1),
        LEFT(strafePower.LEFT,1,-1,-1,1),
        RIGHT(strafePower.RIGHT,-1,1,1,-1),
        FL(strafePower.FL, -1,-1,-1,-1),
        FR(strafePower.FL, -1,-1,-1,-1),
        BL(strafePower.FL, -1,-1,-1,-1),
        BR(strafePower.FL, -1,-1,-1,-1),
        CW(strafePower.TURN),
        CCW(strafePower.TURN);
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
        RIGHT(-1,1,1,-1),
        FL(-1,-1,-1,-1),
        FR(-1,-1,-1,-1),
        BL(-1,-1,-1,-1),
        BR(-1,-1,-1,-1),
        TURN();
        int[] modifiers;
        strafePower(int... mods) {
            modifiers = mods;
        }
    }
}