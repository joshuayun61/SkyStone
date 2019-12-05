package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoCommands extends LinearOpMode {

    private Hardware myHardware;

    private DcMotor FR,BR,FL,BL;
    private Servo intake;
    private BNO055IMU imu;
    private Orientation angles;

    public void runOpMode(){

    }

    /**
     * Create instance of Hardware class with correct telemetry + hardwareMap obj
     * Initialize the Hardware variables from the Hardware class (motors, IMU, servos, etc.)
     * Set own local telemetry obj to one that is used in the auto/teleop class.
     * @param telemetry
     * @param hardwareMap
     */
    public AutoCommands(Telemetry telemetry, HardwareMap hardwareMap) {

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        myHardware = new Hardware(telemetry, hardwareMap);

        myHardware.setupMotors();
        myHardware.setupIMU();
        myHardware.setupServos();

        BR = myHardware.BR;
        BL = myHardware.BL;
        FL = myHardware.FL;
        FR = myHardware.FR;

        intake = myHardware.intake;

        imu = myHardware.imu;
    }

    /**
     * Resets the measurments so that they are starting from 0, useful if you only want to turn relative to current location,
     * rather than having to remeber the original location of the robot.
     */
    private void resetAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Checks which direction to move in,
     */
    private void turnToAngle(int angle, double power) {

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetAngle();

        double left, right;

        //Turns based on angle set at desired power while currentAngle is not matching the target angle.
        if (angle < 0) {
            left = power;
            right = -power;
            FL.setPower(left);
            BL.setPower(left);
            FR.setPower(right);
            BR.setPower(right);
            while (angle > currentRelativeAngle(1)) {}
        }
        else if (angle > 0) {
            left = -power;
            right = power;
            FL.setPower(left);
            BL.setPower(left);
            FR.setPower(right);
            BR.setPower(right);
            while (angle < currentRelativeAngle(1)) {}
        }
        else {
            return;
        }

        //Set power to 0 after the turning
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        resetAngle();
    }

    /**
     * Gets a new orientation, finds the change in angle from the older orientation.
     * @param angleNumber Which angle to use for checking change, will change based on REV hub orientation.
     * @return Returns the change in angle from the starting point of when the angles were last reset (During reset it is set to 0)
     */
    private double currentRelativeAngle(int angleNumber) {
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle;

        switch (angleNumber) {
            case 1:
                changeInAngle = newAngles.firstAngle - angles.firstAngle;
                break;
            case 2:
                changeInAngle = newAngles.secondAngle - angles.secondAngle;
                break;
            case 3:
                changeInAngle = newAngles.thirdAngle - angles.thirdAngle;
                break;
            default:
                changeInAngle = 0;
        }

        if (changeInAngle < -180){
            changeInAngle += 360;
        } else if (changeInAngle > 180){
            changeInAngle -= 360;
        }

        return changeInAngle;
    }

    public void strafe(double power, double inches) {

        int ticks = (int) inchesToTicks(inches);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(-power);
        FL.setPower(power);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy()){
            telemetry.addData("Going to ", ticks);
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.update();
        }
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clear();
    }

    public void forward(double power, double inches){

        int ticks = (int) inchesToTicks(inches);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks);
        FR.setTargetPosition(FR.getCurrentPosition() + ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()){
            telemetry.addData("Going to ", ticks);
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.update();
        }
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clear();
    }

    public long inchesToTicks(double inches) {
        long ticks = 0;

        ticks = Math.round((inches / (Math.PI * 3.937008)) * 537.6);

        return ticks;
    }

    public void openServo() {
        intake.setPosition(0.7);
    }
    public void closeServo() {
        intake.setPosition(0.3);
    }
}
