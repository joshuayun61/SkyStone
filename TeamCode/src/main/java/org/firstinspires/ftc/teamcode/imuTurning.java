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

    BNO055IMU       imu;

    DcMotor         FL,FR,BR,BL;

    int             moves = 0;

    double          facing;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false; //what does this do?
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //Hardware Map with port on rev hub
        FL = hardwareMap.get(DcMotor.class, "FL"); // 1 fl
        FR = hardwareMap.get(DcMotor.class, "BR"); // 3 fr
        BL = hardwareMap.get(DcMotor.class, "BL"); // 0 bl
        BR = hardwareMap.get(DcMotor.class, "FR"); // 2 Br

        //Motor Reversals
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);


        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        imu.initialize(parameters);

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //90 degree turn is about spin(2220);
        //Program here:

        while(opModeIsActive()){
            telemetry.addData("Heading", angles.firstAngle);
            //telemetry.update;
        }




    }

    //forward/reverse with encoders
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
        moves++;
    }

    //Strafe left/right with encoders
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

    //rotate with encoders
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
        moves++;
    }

    //reset the orientation snapshot
    public void resetAngle(){

        //This resets the angles of the IMU, eg: First Angle is now 0 wherever it is
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        facing = 0;
    }

    //get current heading
    public double currentAngle(){

        //This creates a new 'snapshot' of the current angles, to be checked against the resetted angular orientatoin
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - angles.firstAngle;

        //Because of
        if (changeInAngle < -180){

            changeInAngle += 360;

        } else if (changeInAngle > 180){

            changeInAngle -= 360;

        }

        facing += changeInAngle;

        angles = newAngles;

        return facing;

    }

    //  + (Positive) is left turn  - (Negative) is right turn

    //stops and turns to fix heading
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



