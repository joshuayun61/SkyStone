package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract class MotorMethods {

    DcMotor[] motors;

    public boolean isBusy() {
        for (DcMotor motor: motors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for(DcMotor motor: motors) {
            motor.setMode(runMode);
        }
    }

    public void setRunMode(DcMotor.RunMode... runModes) {
        for (DcMotor.RunMode runMode : runModes) {
            for (DcMotor motor : motors) {
                motor.setMode(runMode);
            }
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for(DcMotor motor: motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void addTargetPosition(int count) {
        for (int i = 0; i < motors.length; i++) {
            DcMotor motor = motors[i];
            motor.setTargetPosition(motor.getCurrentPosition() + count);
        }
    }

    public void addTargetPosition(int... count) {
        for (int i = 0; i < motors.length; i++) {
            DcMotor motor = motors[i];
            motor.setTargetPosition(motor.getCurrentPosition() + count[i]);
        }
    }

    public void setTargetPosition(int position) {
        for (DcMotor motor : motors) motor.setTargetPosition(position);
    }

    private void setTargetPosition(int... position) {
        for (int i = 0; i < motors.length; i++) {
            DcMotor motor = motors[i];
            motor.setTargetPosition(position[i]);
        }
    }

    public void setPower(double power) {
        for (DcMotor motor: motors) {
            motor.setPower(power);
        }
    }

    public void setPower(double... power) {
        for (int i = 0; i < motors.length; i++) {
            DcMotor motor = motors[i];
            motor.setPower(power[i]);
        }
    }

    public void initDcMotorArray(DcMotor... motor) {
        motors = new DcMotor[motor.length];

        for (int i = 0; i < motors.length; i++) {
            motors[i] = motor[i];
        }
    }


}