package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;


@TeleOp (name="gyroPivot", group="Iterative Opmode")

public class gyroPivot extends OpMode{
    //Declaring OpMode Members
    private ElapsedTime runTime = new ElapsedTime();
    private CRServo pivotX;
    private ModernRoboticsI2cGyro gyro;


    @Override
    public void init()
    {

        telemetry.addData("MODE:", "initializing...");
        //Hardware mapping
        pivotX = hardwareMap.get(CRServo.class, "xstab");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.calibrate();
        while (gyro.isCalibrating())
        {
            telemetry.addData("MODE:", "Gyro Calibrating...");
            telemetry.addData("MODE:", "DO NOT MOVE");
        }


        telemetry.addData("MODE:", "Initialized");
        telemetry.update();
    }


    @Override
    public void init_loop() {
    }
    @Override
    public void start()
    {
        runTime.reset();
    }

    public void loop()
    {
        int gyroX = gyro.rawX();
        int gyroY = gyro.rawY();
        int gyroZ = gyro.rawZ();
        double heading = gyro.getHeading();

        double pitchError;



        pitchError = 0; //CRServo Speed

        if (heading<180 && heading>0)
        {
            heading = -heading;
        }
        else if (heading>180 && heading<359) {
            heading = 360-heading;
            pitchError = pitchError;
        }

        pitchError = (heading/180); //CRServo Speed

        pivotX.setPower(pitchError);
        telemetry.addData("Heading", heading);
        telemetry.addData("Rotation", pitchError);
        telemetry.update();
    }

    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}



