package org.firstinspires.ftc.teamcode;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Teleop with Abstraction")

public class Teleop extends LinearOpMode
{

    @Override
    public void runOpMode() {

        TeleopCommands commands = new TeleopCommands(telemetry, hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while(opModeIsActive()) {
            commands.mecanum();
            commands.turn180();
            commands.slideMotor();
        }
    }
}
