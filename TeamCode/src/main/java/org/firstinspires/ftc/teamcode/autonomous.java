package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "autonomous", group = "Linear OpMode")

public class autonomous extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AcwrW0v/////AAABma7B+GULfEomjfP2ZL34WDgr9iGtLUtgVA/x6Z7Fi/1DgUg69cGmFmMg2vo1yNWmr3/ZSoJJBmj1ahtA+KNA07v5mAdQIYz7zo1TEENpcIUbHBccVQ12zHjxfeXkNDKhapCU9GxljP7QwdGI5h13beyVZYqKls+pnDKDWAyFAcDhE6cf5xs6jzlyKuiG53qulBiROMhFl1Oo1+zkgWCQhUitxXnHag2LmL6EtnjRFhLDMQ65CPrUNBG9Te8+2K1Na4SCURDWRtlRUJPsKX3O/O7DCVugkeJ01v7/pGUf20nIzrz+7zguCPGXN7a715lOmOmQn7LvTKYklTLJ+Si3Q0rGUaqxmGPt57pEJFvIHDZz";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private int skyStoneLocation = 0;

    @Override
    public void runOpMode()
    {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        while(!opModeIsActive())
        {
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
                    int stone2Left = -1;
                    for (Recognition recognition : updatedRecognitions)
                    {
                        if (recognition.getLabel().equals("LABEL_SECOND_ELEMENT"))
                        {
                            skyLeft = (int) recognition.getLeft();
                        }
                        else if (stone1Left == -1)
                        {
                            stone1Left = (int) recognition.getLeft();
                        }
                        else
                        {
                            stone2Left = (int) recognition.getLeft();
                        }
                        if (skyLeft > stone1Left && stone1Left != -1)
                        {
                            skyStoneLocation = 1;
                        }
                        else if (skyLeft != -1)
                        {
                            skyStoneLocation = 2;
                        }
                        else
                        {
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

        if (tfod != null) {
            tfod.shutdown();
        }

        while(opModeIsActive())
        {

            telemetry.update();
        }
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
}
