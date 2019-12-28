package org.firstinspires.ftc.teamcode.AI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.vuforia.CameraDevice;


public class TensorSense extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public int stonePosition = -100;

    public TensorSense(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }


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

    @Override
    public void runOpMode() {}


    public void setupTfod(){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        CameraDevice.getInstance().setFlashTorchMode(true);
    }

    public void runTfod() {


        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            telemetry.addData("# Objects detected", updatedRecognitions.size());

            int skyStoneLeftX = -1;
            int stoneLeftX = -1;
            int stoneRightX = -1;

            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                    skyStoneLeftX = (int) recognition.getLeft();
                }
                else if (stoneLeftX == -1){
                    stoneLeftX = (int) recognition.getLeft();
                }
                else if (stoneRightX == -1){
                    stoneRightX = (int) recognition.getLeft();

                }

            }


            telemetry.addData("SkyStone LeftX", skyStoneLeftX);
            telemetry.addData("Stone 1LeftX", stoneLeftX);
            telemetry.addData("Stone 2LeftX", stoneRightX);

            /*if (stoneRightX == -1 && skyStoneLeftX < stoneLeftX) {
                telemetry.addData("SkyStone Position", "Left");
                stonePosition = -1;
            } else if (skyStoneLeftX == -1) {
                telemetry.addData("SkyStone Positio", "Right");
                stonePosition = 1;
            } else if (stoneRightX == -1 && stoneLeftX < skyStoneLeftX){
                telemetry.addData("SkyStone Position", "Center");
                stonePosition = 0;
            } else {
                telemetry.addData("SkyStone Position", "Unknown");
            }*/

            if(skyStoneLeftX == -1)
            {
                telemetry.addData("SkyStone Position", "Right");
                stonePosition = 1;
            }
            else if(skyStoneLeftX != -1 && stoneLeftX != -1)
            {
                if(skyStoneLeftX > stoneLeftX)
                {
                    telemetry.addData("SkyStone Position", "Center");
                    stonePosition = 0;
                }
                else
                {
                    telemetry.addData("SkyStone Position", "Left");
                    stonePosition = -1;
                }
            }
            else if(skyStoneLeftX != -1)
            {
                if(skyStoneLeftX > 500)
                {
                    telemetry.addData("SkyStone Position", "Center");
                    stonePosition = 0;
                }
                else
                {
                    telemetry.addData("SkyStone Position", "Left");
                    stonePosition = -1;
                }
            }




        }


        telemetry.update();
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
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
