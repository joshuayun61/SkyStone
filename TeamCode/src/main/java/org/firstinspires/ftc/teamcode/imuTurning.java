/* Ok so
 You can’t just switch between setting power and encoders
 so have fun now that you know that
 ALSO: To go forward:
    FL = +
    BL = +
    BR = -
    FR = -
 ALSO: for the upside down REV hub, the heading is wack
       so, use another hub and map that one and look for the
       comment that says:
*/

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
@Autonomous(name = "imuTurning", group = "Autonomous")

public class imuTurning extends LinearOpMode {



    double facing;


    @Override public void runOpMode() throws InterruptedException {

    }


    //Strafe left/right with encoders


    //reset the orientation snapshot


    //get current heading


    //  + (Positive) is left turn  - (Negative) is right turn

    //stops and turns to fix heading


    //unused :)
    public void facePrecise(int head){
        boolean positiveHead;
        if(head >= 0){
            positiveHead = true;
        } else {
            positiveHead = false;
        }

        if(positiveHead == true && angles.firstAngle < head-10){
            telemetry.update();
            FR.setPower(.4);
            BR.setPower(.4);
            FL.setPower(-.4);
            BL.setPower(-.4);
            telemetry.update();
        } else if(positiveHead == true && angles.firstAngle > head+10){
            telemetry.update();
            FR.setPower(-.4);
            BR.setPower(-.4);
            FL.setPower(.4);
            BL.setPower(.4);
            telemetry.update();
        } else if(angles.firstAngle > head-10 && angles.firstAngle < head+10){
            moves++;
        }

        if(positiveHead == false && angles.firstAngle > head+10){
            telemetry.update();
            FR.setPower(-.4);
            BR.setPower(-.4);
            FL.setPower(.4);
            BL.setPower(.4);
            telemetry.update();
        } else if(positiveHead == false && angles.firstAngle < head-10){
            telemetry.update();
            FR.setPower(.4);
            BR.setPower(.4);
            FL.setPower(-.4);
            BL.setPower(-.4);
            telemetry.update();
        } else if(angles.firstAngle < head-10 && angles.firstAngle > head+10){
            moves++;
        }
    }
    public void faceCont(int head){
        boolean positiveHead;
        if(head > 0){
            positiveHead = true;
        } else {
            positiveHead = false;
        }

        if(positiveHead == true && angles.firstAngle < head-10){
            telemetry.update();
            FR.setPower(.5);
            BR.setPower(.5);
            FL.setPower(-.5);
            BL.setPower(-.5);
            telemetry.addData("heading",head);
            telemetry.update();
        } else if(positiveHead == true && angles.firstAngle > head+10){
            telemetry.update();
            FR.setPower(-.5);
            BR.setPower(-.5);
            FL.setPower(.5);
            BL.setPower(.5);
            telemetry.addData("heading",head);
            telemetry.update();
        } else if(angles.firstAngle > head-10 && angles.firstAngle < head+10){
            //moves++;
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
        }

        if(positiveHead == false && angles.firstAngle > head+10){
            telemetry.update();
            FR.setPower(-.5);
            BR.setPower(-.5);
            FL.setPower(.5);
            BL.setPower(.5);
            telemetry.update();
        } else if(positiveHead == false && angles.firstAngle < head-10){
            telemetry.update();
            FR.setPower(.5);
            BR.setPower(.5);
            FL.setPower(-.5);
            BL.setPower(-.5);
            telemetry.update();
        } else if(angles.firstAngle < head-10 && angles.firstAngle > head+10){
            //moves++;
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
        }
    }
    public void maintainHeading(int head){
        boolean positiveHead;
        if(head > 0){
            positiveHead = true;
        } else {
            positiveHead = false;
        }

        if(positiveHead == true && angles.firstAngle < head-5){
            telemetry.update();
            FR.setPower(.4);
            BR.setPower(.4);
            FL.setPower(.1);
            BL.setPower(.1);
        } else if(positiveHead == true && angles.firstAngle > head+5){
            telemetry.update();
            FR.setPower(.1);
            BR.setPower(.1);
            FL.setPower(.4);
            BL.setPower(.4);
        } else {
            telemetry.update();
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
        }
        if(positiveHead == false && angles.firstAngle > head+5){
            telemetry.update();
            FR.setPower(.1);
            BR.setPower(.1);
            FL.setPower(.4);
            BL.setPower(.4);
        } else if(positiveHead == false && angles.firstAngle < head-5){
            telemetry.update();
            FR.setPower(.4);
            BR.setPower(.4);
            FL.setPower(.1);
            BL.setPower(.1);
        } else {
            telemetry.update();
            FR.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
        }
    }
    public void strafeLeft(double power, int distance){
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(BR.getCurrentPosition() + distance);
        BL.setTargetPosition(BL.getCurrentPosition() - distance);
        FR.setTargetPosition(FR.getCurrentPosition() - distance);
        FL.setTargetPosition(FL.getCurrentPosition() + distance);

        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy()){
            telemetry.addData("Going to ", distance);
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.update();
        }
        moves++;

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}



