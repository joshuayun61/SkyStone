package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.teamcode.NewRobot.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain extends LinearOpMode {

    public DcMotor FL, FR, BL, BR;
    public DcMotor LS, RS;
    Servo rightRepos, leftRepos;
    public Servo RR, LR;
    public DcMotor tapeMeasure;
    public ModernRoboticsI2cRangeSensor rangeSensor;

    public final float Ku = .85f;
    public float Kp = Ku/2;

    boolean reverse = false;

    public boolean powerOn = false;
    private double error;

    DcMotor[] motors = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {}

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


   //     leftRepos = hardwareMap.get(Servo.class, "leftR");
    //    rightRepos = hardwareMap.get(Servo.class, "rightR");

        DcMotor[] tempMotors = {FL, FR, BL, BR};
        motors = tempMotors;
    }
    public DriveTrain(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad, boolean New) {

        gamepad1 = gamepad;

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");


        LS = hardwareMap.get(DcMotor.class, "LS");
        RS = hardwareMap.get(DcMotor.class, "RS");

        tapeMeasure = hardwareMap.get(DcMotor.class, "TM");

        RR = hardwareMap.get(Servo.class, "RR");
        LR = hardwareMap.get(Servo.class, "LR");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);


        RS.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] tempMotors = {FL, FR, BL, BR};
        motors = tempMotors;
    }




    public void mecanumDrive() {

        double drive, strafe, spin;

        if(!reverse) {
             drive = -gamepad1.right_stick_y;
             strafe = -gamepad1.right_stick_x;
             spin = gamepad1.left_stick_x;
        }
        else
        {
             drive = gamepad1.right_stick_y;
             strafe = gamepad1.right_stick_x;
             spin = gamepad1.left_stick_x;
        }

        if(gamepad1.y)
        {
             reverse = true;
        }
        if(gamepad1.b)
        {
            reverse = false;
        }

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

        if(gamepad1.dpad_right)
        {
            tapeIn();
        }
        else if(gamepad1.dpad_left)
        {
            tapeOut();
        }
        else
        {
            tapeMeasure.setPower(0);
        }

        if(gamepad1.a)
        {
            openRepos();
        }

        if(gamepad1.b)
        {
            closeRepos();
        }
        if(gamepad1.left_bumper && gamepad1.dpad_down)
        {
            suckInSlow();
        }

    }
    public void tapeIn()
    {
        tapeMeasure.setPower(-1);
    }
    public void tapeOut()
    {
        tapeMeasure.setPower(1);
    }
    public void suck()
    {

        if(gamepad1.dpad_up)
        {
            suckOut();
        }
        if(gamepad1.dpad_down)
        {
            suckIn();
        }
        if(gamepad1.x)
        {
            suckOff();
        }
        if(gamepad1.left_bumper && gamepad1.dpad_down)
        {
            suckInSlow();
        }
    }

    public void suckOut()
    {

            LS.setPower(.55);
            RS.setPower(.55);

    }
    public void suckOff()
    {

        LS.setPower(0);
        RS.setPower(0);

    }

    public void suckIn()
    {
        LS.setPower(-.6);
        RS.setPower(-.6);
    }
    public void suckInSlow()
    {
        LS.setPower(-.2);
        RS.setPower(-.2);
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

    public void closeRepos()
    {
        LR.setPosition(.2);
        RR.setPosition(.8);
    }

    public void openRepos()
    {
        LR.setPosition(.8);
        RR.setPosition(.3);
    }
//    public double PISpin(int angle,IMU imu, ElapsedTime time)
//    {
//       // while(imu.currentAngle() != angle)
//        //{
//            error = imu.PISend(angle);
//            error += time.time()/20;
//        //}
//        return error;
//    }

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
            while(avgCurrent < ticks - 30)
            {
                //To create a motor speed that increases as it approaches halfway then decreases as it approaches it's target
                //most calculations will be based on ticks/2

                //This converts the current position to a standard score from 0 to 1 that we can manipulate for motor speed.
                double error = (avgCurrent - ticks/2)/ticks/2;

                double power = -(Math.pow(error, 2)) + 1;
                limit(power, .12,.8);

              //  FL.setPower(-power);
              //  BL.setPower(-power);
              //  FR.setPower(-power);
              //  BR.setPower(-power);
                telemetry.addData("Power", power);
                telemetry.addData("AvgCurrent", avgCurrent);
                telemetry.addData("ticsk:", ticks);
                telemetry.addData("Error", error);
                telemetry.update();

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
    public void PropDriveIMU(double distance, double maxSpeed, IMU imu)
    {
        int ticks = inchesToTicks(distance);
        double angle = imu.currentAngle();

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

        while(currentAbs < targetAbs - 20 || currentAbs > targetAbs + 20) {

            // Left = -

                double relativeDistance = FR.getTargetPosition() - FR.getCurrentPosition();
                double power = relativeDistance * Kp / 1000;
                power = limit(power, .15, maxSpeed);
                double correction = angle - imu.currentAngle();
                correction = correction * Kp /10;

                FL.setPower(power + correction);
                BL.setPower(power + correction);
                BR.setPower(power - correction);
                FR.setPower(power - correction);

               // motor.setPower(power);
                telemetry.addData("Left", power + correction);
                telemetry.addData("Right", power - correction);


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

    public void PropDriveIMU(double distance, double maxSpeed, IMU imu, int angle)
    {
        int ticks = inchesToTicks(distance);
        double currentAngle = imu.currentAngle();

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

        while(currentAbs < targetAbs - 20 || currentAbs > targetAbs + 20) {

            // Left = -

            double relativeDistance = FR.getTargetPosition() - FR.getCurrentPosition();
            double power = relativeDistance * Kp / 1000;
            power = limit(power, .15, maxSpeed);
            double correction = angle - imu.currentAngle();
            correction = correction * Kp /10;

            FL.setPower(power + correction);
            BL.setPower(power + correction);
            BR.setPower(power - correction);
            FR.setPower(power - correction);

            // motor.setPower(power);
            telemetry.addData("Left", power + correction);
            telemetry.addData("Right", power - correction);


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

    public void newDrive(double distance, ElapsedTime timePassed, double drift, LinearOpMode runSide)
    {
        double timeNormalize = timePassed.time()/45;

        int avgCurrent = 0;
        for(DcMotor motor: motors) {
            avgCurrent += motor.getCurrentPosition();
        }

        while(runSide.opModeIsActive())
        {

        }
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
    public void turn (double error, boolean isRight, ElapsedTime timePassed)
    {
        double timeNormalize = timePassed.time()/45;
        double power = error + 2*timeNormalize;

        power = limit(power, .08,.8);

        if(isRight)
        {
            FL.setPower(power);
            BL.setPower(power);
            FR.setPower(-power);
            BR.setPower(-power);
        }
        else
        {
            FL.setPower(power);
            BL.setPower(power);
            FR.setPower(-power);
            BR.setPower(-power);
        }

    }

    public void halt()
    {
        for(DcMotor motor: motors)
            motor.setPower(0);
    }

    public double getOptical()
    {
        return rangeSensor.rawOptical();
    }

    public double cmOptical()
    {
        return rangeSensor.cmOptical();
    }

    public double ultrasonic()
    {
         return rangeSensor.rawUltrasonic();
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
        boolean positive;
        if(ticks > 0)
        {
            ticks += 55;
            positive = true;
        }
        else {
            ticks -= 55;
            positive = false;
        }

//        for (DcMotor motor : motors) {
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);


        if(positive) {
            BR.setPower(power);
            BL.setPower(-power);
            FR.setPower(-power);
            FL.setPower(power);
        }
        else
        {
            BR.setPower(-power);
            BL.setPower(power);
            FR.setPower(power);
            FL.setPower(-power);
        }

        if(positive)
        {
            while(FL.getCurrentPosition() < FL.getTargetPosition())
            {
                telemetry.addLine()
                    .addData("Target", FL.getTargetPosition())
                        .addData("Current", FL.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            while(FL.getCurrentPosition() > FL.getTargetPosition())
            {
                telemetry.addLine()
                        .addData("Target", FL.getTargetPosition())
                        .addData("Current", FL.getCurrentPosition());
                telemetry.update();
            }
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


    }
    public void spin(double distance, double power) {

        //default spin in to the left

        //   int ticks = inchesToTicks(distance);
        int ticks = (int)distance;
        boolean positive;
        if(ticks > 0)
        {
            positive = true;
        }
        else
        {
            positive = false;
        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        if(positive) {
            BR.setPower(-power);
            BL.setPower(power);
            FR.setPower(-power);
            FL.setPower(power);
        }
        else
        {
            BR.setPower(power);
            BL.setPower(-power);
            FR.setPower(power);
            FL.setPower(-power);
        }

        if(positive)
        {
            while(FL.getCurrentPosition() < FL.getTargetPosition())
            {
                telemetry.addLine()
                        .addData("Target", FL.getTargetPosition())
                        .addData("Current", FL.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            while(FL.getCurrentPosition() > FL.getTargetPosition())
            {
                telemetry.addLine()
                        .addData("Target", FL.getTargetPosition())
                        .addData("Current", FL.getCurrentPosition());
                telemetry.update();
            }
        }



        for (DcMotor motor: motors) {
            motor.setPower(0);
        }


    }

    public void imuStrafe(int distance, double power, IMU imu)
    {
        int ticks = inchesToTicks(distance);
        boolean positive;
        if(ticks > 0)
        {
            ticks += 255;
            positive = true;
        }
        else {
            ticks -= 255;
            positive = false;
        }

//        for (DcMotor motor : motors) {
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        double startAngle = imu.currentAngle();
        double correction = 0;

        if(positive)
        {
            while(FL.getCurrentPosition() < FL.getTargetPosition())
            {
                correction = startAngle - imu.currentAngle();
                correction = correction * imu.Kp /10;
                    BR.setPower(power - correction);
                    BL.setPower(-power + correction);
                    FR.setPower(-power - correction);
                    FL.setPower(power + correction);

                telemetry.addLine()
                        .addData("Positive Target", FL.getTargetPosition())
                        .addData("Current", FL.getCurrentPosition())
                        .addData("Correction", correction);
                telemetry.update();
            }
        }
        else
        {
            while(FL.getCurrentPosition() > FL.getTargetPosition())
            {
                correction = startAngle - imu.currentAngle();
                correction = correction * imu.Kp /10;
                BR.setPower(-power - correction);
                BL.setPower(power + correction);
                FR.setPower(power - correction);
                FL.setPower(-power + correction);

                telemetry.addLine()
                        .addData(" !Positive Target", FL.getTargetPosition())
                        .addData("Current", FL.getCurrentPosition())
                        .addData("Correction", correction);
                telemetry.update();
            }
        }

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


    }

    public void spline(double power, int inches, int direction)
    {
        int distance = inchesToTicks(inches);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(direction == 45)
        {
            BR.setTargetPosition(-distance);

            while (BR.getCurrentPosition() > BR.getTargetPosition()) {
                FL.setPower(-power);
                BR.setPower(-power);
            }
        }
        else if(direction == 135)
        {
            FR.setTargetPosition(-distance);

            while(FR.getCurrentPosition() > FR.getTargetPosition()) {
                FR.setPower(-power);
                BL.setPower(-power);
            }
        }
        else if(direction == 225) {
            BR.setTargetPosition(distance);

            while (BR.getCurrentPosition() < BR.getTargetPosition()) {
                FL.setPower(power);
                BR.setPower(power);
            }

        }
        else if(direction == 315)
        {
            FR.setTargetPosition(distance);

            while(FR.getCurrentPosition() < FR.getTargetPosition()) {
                FR.setPower(power);
                BL.setPower(power);
            }
        }
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
    }


    public int inchesToTicks(double distance) {

        return ((int)(((distance -1) / (Math.PI * 3.937008)) * 537.6));
    }

    public int ticksToInches (int ticks){
        return (int)(((Math.PI * 3.937008)*ticks) / 537.6) + 1;
    }

}