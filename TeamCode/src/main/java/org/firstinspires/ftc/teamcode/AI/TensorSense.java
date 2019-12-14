package org.firstinspires.ftc.teamcode.AI;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.vuforia.CameraDevice;

@TeleOp(name = "TensorSense", group = "Concept")

public class TensorSense extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    int stonePosition = 0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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
    public void runOpMode() {
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
        waitForStart();


        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.

        List<Recognition> recogs = tfod.getUpdatedRecognitions();

        if(recogs != null)
        {
            //if size is 1, that means out of the first two, the skystone was not found,
            //but it is there. This is known because the success rate of finding two stones
            //is near perfect.
            if(recogs.size() == 1)
            {
                telemetry.addData("SkyStone is in position", 1);
                stonePosition = 1;
            }
            else
            {
                for(int i = 0; i < recogs.size(); i++)
                {
                    if(recogs.get(i).getLabel().equals(LABEL_SECOND_ELEMENT))
                    {
                        telemetry.addData("SkyStone is in position", i+1);
                        stonePosition = i+1;
                    }
                }
            }
            if(stonePosition == 0)
            {
                stonePosition = 3;
            }
        }




        CameraDevice.getInstance().setFlashTorchMode(false);
        tfod.shutdown();

        while(opModeIsActive())
        {
            telemetry.addData("stonePosition", stonePosition);
            telemetry.update();
        }



//                    //    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    if(updatedRecognitions != null) {
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//
//                            telemetry.addData(recognition.getLabel(), null);
//
////                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
////                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
////                                    recognition.getLeft(), recognition.getTop());
////                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
////                                    recognition.getRight(), recognition.getBottom());
//                        }
//                        telemetry.update();
//                    }



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
