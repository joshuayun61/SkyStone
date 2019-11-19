package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@TeleOp (name="intakeTest", group="Iterative Opmode")

public class  intakeTest extends OpMode
{

    int servoPosition = 0;
    Servo intake;

    @Override
    public void init()
    {
        intake = hardwareMap.get(Servo.class,"intake");
    }

    @Override
    public void loop()
    {

        if(gamepad1.b){
            intake.setPosition(.5);
        }
        if(gamepad1.a){
            intake.setPosition(.99);

        }
    }
}
