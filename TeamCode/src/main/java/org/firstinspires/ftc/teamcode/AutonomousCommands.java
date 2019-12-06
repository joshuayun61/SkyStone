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

public class AutonomousCommands extends LinearOpMode {

    private Hardware myHardware;

    private DcMotor FR,BR,FL,BL;
    private Servo intake;
    private IMUCommands imu;
    private Orientation angles;
    private DcMotor[] driveMotors;

    public void runOpMode(){

    }

    /**
     * Create instance of Hardware class with correct telemetry + hardwareMap obj
     * Initialize the Hardware variables from the Hardware class (motors, IMUCommands, servos, etc.)
     * Set own local telemetry obj to one that is used in the auto/teleop class.
     * @param telemetry
     * @param hardwareMap
     */
    public AutonomousCommands(Telemetry telemetry, HardwareMap hardwareMap) {

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        myHardware = new Hardware(telemetry, hardwareMap);

        myHardware.setupMotors();
        myHardware.setupIMU();
        myHardware.setupServos();

        DcMotor[] tempMotors = {myHardware.BR, myHardware.BL, myHardware.FR, myHardware.FL};

        BR = myHardware.BR;
        BL = myHardware.BL;
        FL = myHardware.FL;
        FR = myHardware.FR;

        driveMotors = tempMotors;

        intake = myHardware.intake;

        imu = new IMUCommands(telemetry, hardwareMap);
    }


    public void forward(double power, double inches){

        int ticks = (int) inchesToTicks(inches);

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        BR.setTargetPosition(BR.getCurrentPosition() + ticks);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks);
        FR.setTargetPosition(FR.getCurrentPosition() + ticks);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks);

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);



        while(BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()){
            telemetry.addData("Going to ", ticks);
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.update();
        }

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.clear();
    }

    public void strafe(double power, double inches) {
        int ticks = (int) inchesToTicks(inches);

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        BR.setTargetPosition(BR.getCurrentPosition() - ticks);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks);
        FR.setTargetPosition(FR.getCurrentPosition() + ticks);
        FL.setTargetPosition(FL.getCurrentPosition() - ticks);

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        BR.setPower(-power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(-power);



        while(BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()){
            telemetry.addData("Going to ", ticks);
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.update();
        }

        for(DcMotor motor: driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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
