package org.firstinspires.ftc.teamcode;

import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Temporary Mecanum Drive")

public class Teleop extends OpMode
{


    @Override
    public void init() {

        //Instantiate local class instances
        Hardware hardware = new Hardware(telemetry, hardwareMap);
        hardware.setup();


    }


   
    @Override
    public void loop() {
        

        if(gamepad1.y)
        {
            turnToAngle(180,.7);
        }
        if(gamepad1.a)
        {
            Slide.setTargetPosition(Slide.getCurrentPosition() + 3);
            Slide.setPower(.4);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad1.b)
        {
            Slide.setTargetPosition(Slide.getCurrentPosition() - 3);
            Slide.setPower(.4);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    
    @Override
    public void stop() {
    }
    
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
