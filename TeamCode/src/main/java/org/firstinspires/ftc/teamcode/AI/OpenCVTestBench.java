package org.firstinspires.ftc.teamcode.AI;

import org.firstinspires.ftc.teamcode.Robot12382;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;

import java.util.List;
import java.util.ArrayList;

public class OpenCVTestBench {

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;


    OpenCvCamera phoneCam;

    public double getColor() {
        return valMid;
    }

    public void setup() throws InterruptedException {

        int cameraMonitorViewId = Robot12382.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", Robot12382.hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new ColorDetect());//different stages
        //width, height
        //width = height in this case, because camera is in portrait mode.
    }

    public void start() {
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
    }

    public void stop() {
        phoneCam.stopStreaming();
    }


    static class ColorDetect extends OpenCvPipeline {

        Mat hsv = new Mat();
        Mat red1 = new Mat();
        Mat red2 = new Mat();
        Mat redFinal = new Mat();
        Mat blur = new Mat();

        enum Stage
        {
            RAW_IMAGE,
            BLURRED,
            RED
        }

        private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.GaussianBlur(input, blur, new Size(5,5),0);

            Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsv, new Scalar(0,50,50), new Scalar(10,255,255), red1);
            Core.inRange(hsv, new Scalar(170,50,50), new Scalar(180,255,255), red2);

            Core.bitwise_or(red1, red2, redFinal);

            double[] pixMid = redFinal.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));

            Imgproc.circle(input, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //Returns stages of the pipeline based on how many screen presses
            switch (stageToRenderToViewport) {
                case RED:
                    return redFinal;
                case BLURRED:
                    return blur;
                case RAW_IMAGE:
                    return input;
                default:
                    return null;
            }

        }
    }
}
