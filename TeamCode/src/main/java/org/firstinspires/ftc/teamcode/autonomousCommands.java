package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class autonomousCommands extends LinearOpMode {

    hardware hardware = new hardware();


    DcMotor BR = hardware.BR;
    DcMotor BL = hardware.BL;
    DcMotor FL = hardware.FL;
    DcMotor FR = hardware.FR;

    public void runOpMode(){

    }

    public void resetAngle(){

        //This resets the angles of the IMU, eg: First Angle is now 0 wherever it is
        hardware.angles   = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        hardware.facing = 0;
    }

    public double currentAngle(){

        //This creates a new 'snapshot' of the current angles, to be checked against the resetted angular orientatoin
        Orientation newAngles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - hardware.angles.firstAngle;

        //Because of
        if (changeInAngle < -180){

            changeInAngle += 360;

        } else if (changeInAngle > 180){

            changeInAngle -= 360;

        }

        hardware.facing += changeInAngle;

        hardware.angles = newAngles;

        return hardware.facing;

    }

    public void face(int head, double power){

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Because we want to spin, only two powers are needed
        double left, right;

        resetAngle();

        if (head < 0){   // turns right

            left = power;
            right = -power;

        }
        else if (head > 0){   // turns left

            left = -power;
            right = power;

        } else {

            return;

        }

        FL.setPower(left);
        BL.setPower(left);
        FR.setPower(right);
        BR.setPower(right);

        if (head < 0){

            /* Test Need for this line:
            while (opModeIsActive() && currentAngle() == 0) {}
            */

            //right
            while (opModeIsActive() && currentAngle() > head){
                //continue turning
            }
        } else {

            // left
            while (opModeIsActive() && currentAngle() < head) {
                //continue turning
            }
        }

        //Stop after turn
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);

        // reset angle tracking on new heading.
        resetAngle();

    }

    public void forward(double power, int distance){

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(BR.getCurrentPosition() + distance);
        BL.setTargetPosition(BL.getCurrentPosition() + distance);
        FR.setTargetPosition(FR.getCurrentPosition() + distance);
        FL.setTargetPosition(FL.getCurrentPosition() + distance);

        BR.setPower(-power);
        BL.setPower(power);
        FR.setPower(-power);
        FL.setPower(power);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()){
            telemetry.addData("Going to ", distance);
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

    public void strafe(double power, int distance){
        //by default a strafe is left
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(BR.getCurrentPosition() - distance);
        BL.setTargetPosition(BL.getCurrentPosition() + distance);
        FR.setTargetPosition(FR.getCurrentPosition() + distance);
        FL.setTargetPosition(FL.getCurrentPosition() - distance);

        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy()) {
            telemetry.addData("Going to ", distance);
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.update();
        }
    }

    public void spin(double power, int degrees){

        int distance = (int)Math.round(25.7*degrees);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(BR.getCurrentPosition() + distance);
        BL.setTargetPosition(BL.getCurrentPosition() + distance);
        FR.setTargetPosition(FR.getCurrentPosition() + distance);
        FL.setTargetPosition(FL.getCurrentPosition() + distance);

        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()){
            telemetry.addData("Going to ", distance);
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
}
