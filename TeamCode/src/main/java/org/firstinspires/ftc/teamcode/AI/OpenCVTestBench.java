package org.firstinspires.ftc.teamcode.AI;

import org.firstinspires.ftc.teamcode.Robot12382;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVTestBench {

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;


    public void setup() throws InterruptedException {

        int cameraMonitorViewId = Robot12382.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", Robot12382.hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new OpenCV.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

    }

    static class SwitchingPipeline extends OpenCvPipeline {

        enum Stage
        {

        }

        @Override
        public void onViewportTapped() {

        }

        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }
}
