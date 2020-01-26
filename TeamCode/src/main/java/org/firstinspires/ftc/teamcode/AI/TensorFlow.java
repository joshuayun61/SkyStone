package org.firstinspires.ftc.teamcode.AI;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot12382;

import com.vuforia.CameraDevice;


public class TensorFlow {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public int stonePosition = -1;
    public int SkyStoneLeftBound = 0;


    private static final String VUFORIA_KEY =
            "AcwrW0v/////AAABma7B+GULfEomjfP2ZL34WDgr9iGtLUtgVA/x6Z7Fi/1DgUg69cGmFmMg2vo1yNWmr3/ZSoJJBmj1ahtA+KNA07v5mAdQIYz7zo1TEENpcIUbHBccVQ12zHjxfeXkNDKhapCU9GxljP7QwdGI5h13beyVZYqKls+pnDKDWAyFAcDhE6cf5xs6jzlyKuiG53qulBiROMhFl1Oo1+zkgWCQhUitxXnHag2LmL6EtnjRFhLDMQ65CPrUNBG9Te8+2K1Na4SCURDWRtlRUJPsKX3O/O7DCVugkeJ01v7/pGUf20nIzrz+7zguCPGXN7a715lOmOmQn7LvTKYklTLJ+Si3Q0rGUaqxmGPt57pEJFvIHDZz";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;



    public void setupTfod(){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            Robot12382.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        Robot12382.telemetry.addData(">", "Press Play to start op mode");
        Robot12382.telemetry.update();
        CameraDevice.getInstance().setFlashTorchMode(true);
    }

    public void runTfod() {


        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            Robot12382.telemetry.addData("# Objects detected", updatedRecognitions.size());

            int skyStoneLeftX = -1;
            int stoneLeftX = -1;
            int stoneRightX = -1;

            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                    skyStoneLeftX = (int) recognition.getLeft();
                    SkyStoneLeftBound = skyStoneLeftX;
                } else if (stoneLeftX == -1) {
                    stoneLeftX = (int) recognition.getLeft();
                } else if (stoneRightX == -1) {
                    stoneRightX = (int) recognition.getLeft();

                }


                Robot12382.telemetry.addData("SkyStone LeftX", skyStoneLeftX);
                Robot12382.telemetry.addData("Stone 1LeftX", stoneLeftX);
                Robot12382.telemetry.addData("Stone 2LeftX", stoneRightX);


                if (skyStoneLeftX == -1 && stoneLeftX != 0 && stoneRightX != 0) {
                    Robot12382.telemetry.addData("SkyStone Position", "Right");
                    stonePosition = 3;
                } else if (skyStoneLeftX != -1 && stoneLeftX != -1) {
                    if (skyStoneLeftX > stoneLeftX) {
                        Robot12382.telemetry.addData("SkyStone Position", "Center");
                        stonePosition = 2;
                    } else {
                        Robot12382.telemetry.addData("SkyStone Position", "Left");
                        stonePosition = 1;
                    }
                } else if (skyStoneLeftX != -1) {
                    if (skyStoneLeftX > 500) {
                        Robot12382.telemetry.addData("SkyStone Position", "Center");
                        stonePosition = 2;
                    } else {
                        Robot12382.telemetry.addData("SkyStone Position", "Left");
                        stonePosition = 1;
                    }
                }
            }



        }


        Robot12382.telemetry.update();
    }

    public void stopTfod() {
        CameraDevice.getInstance().setFlashTorchMode(false);
        tfod.shutdown();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = Robot12382.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", Robot12382.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}