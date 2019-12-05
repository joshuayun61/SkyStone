package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class TensorMethods extends LinearOpMode {

    private int skyStoneLocation = 0;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AcwrW0v/////AAABma7B+GULfEomjfP2ZL34WDgr9iGtLUtgVA/x6Z7Fi/1DgUg69cGmFmMg2vo1yNWmr3/ZSoJJBmj1ahtA+KNA07v5mAdQIYz7zo1TEENpcIUbHBccVQ12zHjxfeXkNDKhapCU9GxljP7QwdGI5h13beyVZYqKls+pnDKDWAyFAcDhE6cf5xs6jzlyKuiG53qulBiROMhFl1Oo1+zkgWCQhUitxXnHag2LmL6EtnjRFhLDMQ65CPrUNBG9Te8+2K1Na4SCURDWRtlRUJPsKX3O/O7DCVugkeJ01v7/pGUf20nIzrz+7zguCPGXN7a715lOmOmQn7LvTKYklTLJ+Si3Q0rGUaqxmGPt57pEJFvIHDZz";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {}

    /**
     * Setup for the telemetry and hardwaremappings since should only have 1 instance of telemetry being used between all the classes.
     * @param telemetry Pass the telemetry object of the main Autonomous12382/teleop class to prevent conflict between the two during telemetry.add___()
     * @param hardwareMap Same as above but for Hardware mapping
     */
    public TensorMethods(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    /**
     * Calls the initTfod and initVuforia to prepare for object detection
     */
    public void setup() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /**
     * Updates the list of recognitions it is getting from the Camera
     * Iterates through the list of recognitions and determine with logic, where the skystone position is.
     */
    public void runTfod() {
        if (tfod != null) {
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

    /**
     * Stops the tfod instance when necessary to prevent CPU overuse
     */
    public void stopTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
