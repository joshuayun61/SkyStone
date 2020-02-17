package org.firstinspires.ftc.teamcode;

import android.sax.Element;
import android.sax.TextElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AI.TensorSense;
import org.firstinspires.ftc.teamcode.AI.VufandTensor;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.IMU;
import org.firstinspires.ftc.teamcode.Robot.InetialMUnit;
import org.firstinspires.ftc.teamcode.Robot.Arm;

@Autonomous(name = "Methods Test")
public class MethodTests extends LinearOpMode {

    ElapsedTime etime = new ElapsedTime();

    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(telemetry, hardwareMap, gamepad1, true);
      //  TensorSense sense = new TensorSense(telemetry, hardwareMap);
         IMU imu = new IMU(telemetry, hardwareMap, driveTrain);
       // Arm arm = new Arm(telemetry, hardwareMap, gamepad1, driveTrain);
        //sense.setupTfod();
        imu.imuSetup();

        waitForStart();

        driveTrain.PropDriveIMU(20,.6, imu);

    }
}
