package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VuforiaNavigation extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY = "AcwrW0v/////AAABma7B+GULfEomjfP2ZL34WDgr9iGtLUtgVA/x6Z7Fi/1DgUg69cGmFmMg2vo1yNWmr3/ZSoJJBmj1ahtA+KNA07v5mAdQIYz7zo1TEENpcIUbHBccVQ12zHjxfeXkNDKhapCU9GxljP7QwdGI5h13beyVZYqKls+pnDKDWAyFAcDhE6cf5xs6jzlyKuiG53qulBiROMhFl1Oo1+zkgWCQhUitxXnHag2LmL6EtnjRFhLDMQ65CPrUNBG9Te8+2K1Na4SCURDWRtlRUJPsKX3O/O7DCVugkeJ01v7/pGUf20nIzrz+7zguCPGXN7a715lOmOmQn7LvTKYklTLJ+Si3Q0rGUaqxmGPt57pEJFvIHDZz";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    VuforiaTrackables targetsSkyStone;
    List<VuforiaTrackable> allTrackables;

    VuforiaTrackable stoneTarget;
    VuforiaTrackable red1;
    VuforiaTrackable red2;
    VuforiaTrackable front1;
    VuforiaTrackable front2;
    VuforiaTrackable blue1;
    VuforiaTrackable blue2;
    VuforiaTrackable rear1;
    VuforiaTrackable rear2;

    public VuforiaNavigation(Telemetry telemetry, HardwareMap hardwaremap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwaremap;
    }

    @Override
    public void runOpMode() throws InterruptedException {}

    public void vuforiaSetup() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    public void addFieldObjects() {

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");


        allTrackables = new ArrayList<VuforiaTrackable>();

        allTrackables.addAll(targetsSkyStone);
    }

    public void setFieldObjectLocations() {

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    }



}