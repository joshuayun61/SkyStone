package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain extends LinearOpMode {

    public DcMotor FL, FR, BL, BR;
    Servo rightRepos, leftRepos;

    DcMotor[] motors = new DcMotor[4];


    private final float Ku = 2f;
    private float Kp = Ku/2;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public DriveTrain(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad) {

        gamepad1 = gamepad;

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        leftRepos = hardwareMap.get(Servo.class, "leftR");
        rightRepos = hardwareMap.get(Servo.class, "rightR");

        DcMotor[] tempMotors = {FL, FR, BL, BR};
        motors = tempMotors;
    }

    public void mecanumDrive() {

        double drive    = gamepad1.right_stick_y;
        double strafe   = -gamepad1.right_stick_x;
        double spin     = gamepad1.left_stick_x;

        if(gamepad1.right_bumper)
        {
            FL.setPower(drive - strafe - spin);
            FR.setPower(drive + strafe + spin);
            BL.setPower(drive + strafe - spin);
            BR.setPower(drive - strafe + spin);

        }
        else if(gamepad1.left_bumper)
        {
            FL.setPower((drive - strafe - spin)/6);
            FR.setPower((drive + strafe + spin)/6);
            BL.setPower((drive + strafe - spin)/6);
            BR.setPower((drive - strafe + spin)/6);
        }
        else
        {
            FL.setPower((drive - strafe - spin)/3);
            FR.setPower((drive + strafe + spin)/3);
            BL.setPower((drive + strafe - spin)/3);
            BR.setPower((drive - strafe + spin)/3);
        }

        if(gamepad1.right_trigger > 0)
        {
            slowStrafeRight();
        }
        if(gamepad1.left_trigger > 0)
        {
            slowStrafeleft();
        }

        if (gamepad1.a) {
            reposOpen();
        }
        if(gamepad1.b){
            reposClose();
        }


    }

    public void reposClose()
    {
        leftRepos.setPosition(.42);
        rightRepos.setPosition(.6);
    }
    public void reposOpen()
    {
        leftRepos.setPosition(.7);
        rightRepos.setPosition(.35);
    }
    public void slowStrafeRight() {


            double newStrafe = .3;
            FL.setPower(newStrafe);
            FR.setPower(-newStrafe);
            BR.setPower(newStrafe);
            BL.setPower(-newStrafe);

    }
    public void slowStrafeleft()
    {
            double newStrafe = .3;
            FL.setPower(-newStrafe);
            FR.setPower(newStrafe);
            BR.setPower(-newStrafe);
            BL.setPower(newStrafe);

    }

    public void drive(double distance, double power) {

        int ticks = inchesToTicks(distance);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() - ticks);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("EncoderPosition", FL.getCurrentPosition());
            telemetry.addData("EncoderTarget", ticks);
            telemetry.update();
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void PDrive(double inches)
    {
        //convert the distance in inches to ticks
        int ticks = inchesToTicks(inches);
        boolean forward;
        if(ticks > 0)
            forward = true;
        else
            forward = false;

        //Set all motors to run without encoders
        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //We'll stop when the average of all the motors is at the target. This is experimental may need to be reverted, but it purpose
        //is to not just read off one motor for the while loop.
        double avgCurrent = (FL.getCurrentPosition() + BL.getCurrentPosition() + BR.getCurrentPosition() + FR.getCurrentPosition()) / 4;

        if(forward)
        {
            while(avgCurrent < ticks)
            {
                //To create a motor speed that increases as it approaches halfway then decreases as it approaches it's target
                //most calculations will be based on ticks/2

                //This converts the current position to a standard score from 0 to 1 that we can manipulate for motor speed.
                double error = (avgCurrent - ticks/2)/ticks/2;

                double power = -(Math.pow(error, 2)) + 1;

                FL.setPower(power);
                BL.setPower(power);
                FR.setPower(power);
                BR.setPower(power);

                avgCurrent = (FL.getCurrentPosition() + BL.getCurrentPosition() + BR.getCurrentPosition() + FR.getCurrentPosition()) / 4;
            }
        }
        else
        {
            while(avgCurrent > ticks)
            {
                //To create a motor speed that increases as it approaches halfway then decreases as it approaches it's target
                //most calculations will be based on ticks/2

                //This converts the current position to a standard score from 0 to 1 that we can manipulate for motor speed.
                double error = (avgCurrent - (ticks/2))/(ticks/2);

                double power = -(Math.pow(error, 2)) + 1;

                FL.setPower(power);
                BL.setPower(power);
                FR.setPower(power);
                BR.setPower(power);

                avgCurrent = (FL.getCurrentPosition() + BL.getCurrentPosition() + BR.getCurrentPosition() + FR.getCurrentPosition()) / 4;
            }
        }
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

    }



    public void PropDrive(double distance, double maxSpeed)
    {
        int ticks = inchesToTicks(distance);

        int toPower = (Integer.toString(Math.abs(ticks))).length();
        toPower = (int)Math.pow(10, toPower);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() - ticks);

        //convert to abs to reference distance
        double currentAbs = Math.abs(FR.getCurrentPosition());
        double targetAbs = Math.abs(FR.getTargetPosition());

        while(currentAbs < targetAbs - 30 || currentAbs > targetAbs + 30) {


            for(DcMotor motor : motors)
            {
                double relativeDistance = motor.getTargetPosition() - motor.getCurrentPosition();
                double power = relativeDistance * Kp / 1000;
                power = limit(power, .15, maxSpeed);
                motor.setPower(power);
                telemetry.addData("Power", power);
            }

            telemetry.addLine()
                    .addData("Target", BL.getTargetPosition())
                    .addData("Current", BL.getCurrentPosition());
            telemetry.update();

            currentAbs = Math.abs(FR.getCurrentPosition());
            targetAbs = Math.abs(FR.getTargetPosition());
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

    }
    public double limit(double input, double lim, double lim2)
    {
        if(input > 0) {
            if (input < lim) {
                return lim;
            }
            else if(input > lim2)
            {
                return lim2;
            }
        }
        else if(input < 0)
        {
            if(input > 0 - lim)
            {
                return 0 - lim;
            }
            else if(input < 0 - lim2)
            {
                return 0 - lim2;
            }
        }

        return input;
    }


    public void driveAndArm(double distance, double power, Servo gripper, boolean open) {

        int ticks = inchesToTicks(distance);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() - ticks);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("EncoderPosition", FL.getCurrentPosition());
            telemetry.addData("EncoderTarget", ticks);
            telemetry.update();
            if(!open){
                gripper.setPosition(.2);
            }
            else {
                gripper.setPosition(.9);
            }
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void strafe(double distance, double power) {

        int ticks = inchesToTicks(distance);
        if(ticks > 0)
        {
            ticks += 55;
        }
        else {
            ticks -= 55;
        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);


        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("EncoderPosition", FL.getCurrentPosition());
            telemetry.addData("EncoderTarget", ticks);
            telemetry.update();
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void spin(double distance, double power) {

        //default spin in too the left

     //   int ticks = inchesToTicks(distance);
        int ticks = (int)distance;

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("EncoderPosition", FR.getCurrentPosition());
            telemetry.addData("EncoderTarget", FR.getTargetPosition());
            telemetry.update();
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public int inchesToTicks(double distance) {

        return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6));
    }

    public int ticksToInches (int ticks){
        return (int)(((Math.PI * 3.937008)*ticks) / 537.6) + 1;
    }

}
