package org.firstinspires.ftc.teamcode.NewRobot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrainV3 extends LinearOpMode {

    //Declaration of Class Objects
    DcMotor FL, FR, BL, BR; //Powered Wheel Motors
    DcMotor LS, RS;         //Left and Right Suck Motors
    DcMotor tapeMeasure;    //Tape Measure Motor
    DcMotor[] motors = {FL,FR,BR,BL};

    Odometry odometry;

    public BNO055IMU imu;


    //Initializaton of Class Variables
    public Orientation originalAngles;
    public final float Ku = .75f;
    public float Kp = Ku/2;

    //Class Methods
    public void runOpMode() throws InterruptedException {}

    public DriveTrainV3(Telemetry telemetry, HardwareMap hardwareMap, Odometry odometry, Gamepad gamepad) {

        gamepad1 = gamepad;

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;

        this.odometry = odometry;

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

//        LS = hardwareMap.get(DcMotor.class, "LS");
//        RS = hardwareMap.get(DcMotor.class, "RS");

        tapeMeasure = hardwareMap.get(DcMotor.class, "TM");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        //RS.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] tempMotors = {FL, FR, BL, BR};
        motors = tempMotors;

        imuSetup();
    }

    public void imuSetup()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false; //what does this do?

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        originalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }

    public void drive(int ticks, double power)
    {
        for(DcMotor motor: motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(odometry.getYDisplacement() < ticks)
        {
            for(DcMotor motor: motors)
                motor.setPower(power);
            telemetry.addLine()
                    .addData("Current", odometry.getYDisplacement())
                    .addData("Target", ticks);
            telemetry.update();
        }
        for(DcMotor motor: motors)
            motor.setPower(0);
    }

//    public void PropDriveIMU(double distance, double maxSpeed)
//    {
//        int ticks = inchesToTicks(distance);
//        double angle = imu.currentAngle();
//
//        int toPower = (Integer.toString(Math.abs(ticks))).length();
//        toPower = (int)Math.pow(10, toPower);
//
//        for (DcMotor motor : motors) {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        for (DcMotor motor : motors) {
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//
//        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
//        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
//        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
//        FL.setTargetPosition(FL.getCurrentPosition() - ticks);
//
//        //convert to abs to reference distance
//        double currentAbs = Math.abs(FR.getCurrentPosition());
//        double targetAbs = Math.abs(FR.getTargetPosition());
//
//        while(currentAbs < targetAbs - 20 || currentAbs > targetAbs + 20) {
//
//            // Left = -
//
//            double relativeDistance = FR.getTargetPosition() - FR.getCurrentPosition();
//            double power = relativeDistance * Kp / 1000;
//            power = limit(power, .15, maxSpeed);
//            double correction = angle - imu.currentAngle();
//            correction = correction * Kp /10;
//
//            FL.setPower(power + correction);
//            BL.setPower(power + correction);
//            BR.setPower(power - correction);
//            FR.setPower(power - correction);
//
//            // motor.setPower(power);
//            telemetry.addData("Left", power + correction);
//            telemetry.addData("Right", power - correction);
//
//
//            telemetry.addLine()
//                    .addData("Target", BL.getTargetPosition())
//                    .addData("Current", BL.getCurrentPosition());
//            telemetry.update();
//
//            currentAbs = Math.abs(FR.getCurrentPosition());
//            targetAbs = Math.abs(FR.getTargetPosition());
//        }
//
//        FL.setPower(0);
//        BL.setPower(0);
//        FR.setPower(0);
//        BR.setPower(0);
//
//    }

    public void turn(int angle, boolean phoneSide)
    {
        for (DcMotor motor : motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double left, right;

            while (getHeading() > angle + .1 || getHeading() < angle - .1)
            {
                    double angleDifference = angle - getHeading();

                    left = angleDifference * Kp / 100;
                    right = angleDifference * Kp / 100;

                    left = limit(left, .2, 5);
                    right = limit(right, .2, .5);

                    if(phoneSide) {
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(-right);
                        BR.setPower(-right);
                    }
                    else {
                        FL.setPower(-left);
                        BL.setPower(-left);
                        FR.setPower(right);
                        BR.setPower(right);
                    }
            }
            for (DcMotor motor : motors)
                motor.setPower(0);
    }


    /*  Limit method used to ensure that an inputted value is between max and mininum inputted values.
        This method is designed to work for motor powers, if positives are entered with a negative value
        it's all good.
        @param input    - the value to be changed if necessary
        @param min      - minimum value to be returned if the input is below
        @param max      - maximum value to be returned if the input is above
     */
    public double limit(double input, double min, double max) {
        if (input > 0) {
            input += min;
            if (input > max) {
                input = max;
            }
        } else {
            input -= min;
            if (input > -min) {
                input = -min;
            }
        }
        return input;
    }


    /*
             0
        90      -90
            180
     */
    public double getHeading()
    {
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - originalAngles.firstAngle;

        if (changeInAngle < -180){
            changeInAngle += 360;
        } else if (changeInAngle > 180){
            changeInAngle -= 360;
        }

        return changeInAngle;
    }


    public void halt()
    {
        for(DcMotor motor: motors)
            motor.setPower(0);
    }

    public void turn (double error, boolean isRight, ElapsedTime timePassed)
    {
        double timeNormalize = timePassed.time()/100;
        double power = error + timeNormalize;

        power = limit(power, .07,.8);

        if(isRight)
        {
            FL.setPower(power);
            BL.setPower(power);
            FR.setPower(-power);
            BR.setPower(-power);
        }
        else
        {
            FL.setPower(-power);
            BL.setPower(-power);
            FR.setPower(power);
            BR.setPower(power);
        }

    }

}
