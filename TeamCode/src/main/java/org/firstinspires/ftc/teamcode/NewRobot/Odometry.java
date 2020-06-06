package org.firstinspires.ftc.teamcode.NewRobot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odometry extends LinearOpMode {

    //Declaration of Class Objects
    public DcMotor yWheel;

    public ModernRoboticsI2cRangeSensor range;

    //Initializaton of Class Variables

    public Odometry(Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        yWheel = hardwareMap.get(DcMotor.class, "yOdo");
        yWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yWheel.setDirection(DcMotor.Direction.REVERSE);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

    public void runOpMode() {}

    public int getYDisplacement()
    {
        return yWheel.getCurrentPosition();
    }

    public int getUltraSonic()
    {
        return (int)(range.rawUltrasonic()/2.54) + 2;
    }

    public int getOptical()
    {
        return range.rawOptical();
    }

    public double toInches(double input)
    {
        return (2.1698*(input) - 1.886);
    }

}
