package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "autonomous", group = "Linear OpMode")

public class autonomous extends LinearOpMode
{

    /*
     * Vision (Tfod and Vuforia) assets to be used in the init phase of the competition
     */

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AcwrW0v/////AAABma7B+GULfEomjfP2ZL34WDgr9iGtLUtgVA/x6Z7Fi/1DgUg69cGmFmMg2vo1yNWmr3/ZSoJJBmj1ahtA+KNA07v5mAdQIYz7zo1TEENpcIUbHBccVQ12zHjxfeXkNDKhapCU9GxljP7QwdGI5h13beyVZYqKls+pnDKDWAyFAcDhE6cf5xs6jzlyKuiG53qulBiROMhFl1Oo1+zkgWCQhUitxXnHag2LmL6EtnjRFhLDMQ65CPrUNBG9Te8+2K1Na4SCURDWRtlRUJPsKX3O/O7DCVugkeJ01v7/pGUf20nIzrz+7zguCPGXN7a715lOmOmQn7LvTKYklTLJ+Si3Q0rGUaqxmGPt57pEJFvIHDZz";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    /*
     * Hardware variables
     */
    BNO055IMU imu;
    DcMotor FL,FR,BL,BR;
    Servo grabber;

    /*
     * Gyroscope Data
     */
    double facing;
    Orientation angles;
    Acceleration gravity;


    private int skyStoneLocation = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {

        IMUSetupAndMapping();

        hardwareMapping();

        setupTfodVuforia();

        while(!opModeIsActive())
        {
            runTfod();
        }

        stopTfod();

        while(opModeIsActive())
        {
            if (skyStoneLocation == 1) {

            }
            else if (skyStoneLocation == 2) {

            }
            else {

            }
            telemetry.update();
        }
    }

    private void skyStoneLeft() {

    }

    private void skyStoneRight() {

    }

    private void skyStoneMiddle() {

    }

    private void IMUSetupAndMapping() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false; //what does this do?
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.initialize(parameters);

        //Check IMU calibration
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

    }

    private void IMUResetAngle() {
        //This resets the angles of the IMU, eg: First Angle is now 0 wherever it is
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        facing = 0;
    }

    private double snapshotCurrentAngle() {
        //This creates a new 'snapshot' of the current angles, to be checked against the resetted angular orientatoin
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - angles.firstAngle;

        if (changeInAngle < -180){

            changeInAngle += 360;

        } else if (changeInAngle > 180){

            changeInAngle -= 360;

        }

        facing += changeInAngle;

        angles = newAngles;

        return facing;
    }


    private void forwardEncoders() {

    }

    private void hardwareMapping() {

        //Initializing the hardware variables to the configuration mappings in the App
        FL = hardwareMap.get(DcMotor.class, "FL"); // 1 fl
        FR = hardwareMap.get(DcMotor.class, "BR"); // 3 fr
        BL = hardwareMap.get(DcMotor.class, "BL"); // 0 bl
        BR = hardwareMap.get(DcMotor.class, "FR"); // 2 Br

        //Motor Reversals so positive is the same rotation for all motors.
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
    }




    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void setupTfodVuforia() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }
    }

    private void runTfod() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                int skyLeft = -1;
                int stone1Left = -1;

                for (Recognition recognition : updatedRecognitions) {

                    if (recognition.getLabel().equals("LABEL_SECOND_ELEMENT")) {
                        skyLeft = (int) recognition.getLeft();
                    }
                    else if (stone1Left == -1) {
                        stone1Left = (int) recognition.getLeft();
                    }


                    if (skyLeft > stone1Left && stone1Left != -1) {
                        skyStoneLocation = 1;
                    }
                    else if (skyLeft != -1) {
                        skyStoneLocation = 2;
                    }
                    else {
                        skyStoneLocation = 3;
                    }
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    telemetry.addData("SkyStone Location", skyStoneLocation);
                }
                telemetry.update();
            }
        }
    }

    private void stopTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }

    }
}
