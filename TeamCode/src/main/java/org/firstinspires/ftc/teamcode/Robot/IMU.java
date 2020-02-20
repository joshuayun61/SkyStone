package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class IMU extends LinearOpMode {

    DcMotor FL,FR,BL,BR, slide;
    BNO055IMU imu;
    Orientation angles, originalAngles;
    Acceleration gravity;
    DriveTrain driveTrain;
    public final float Ku = .85f;
    public float Kp = Ku/2;
    @Override
    public void runOpMode() throws InterruptedException {}

    public IMU(Telemetry telemetry, HardwareMap hardwareMap, DriveTrain myDriveTrain) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveTrain = myDriveTrain;
        FL = driveTrain.FL;
        FR = driveTrain.FR;
        BL = driveTrain.BL;
        BR = driveTrain.BR;
    }

    public IMU(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    /*  IMU Setup Method used to initialize IMU tracking in any class that the IMU is intended to be used.
        Precondition    - IMU has never been used in the class, otherwise the original values will be overwritten.
        Postcondition   - IMU is ready to be used with any methods in this class.
     */
    public void imuSetup()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU; // added new
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false; //what does this do?
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        originalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /*  Proportional IMU method used to turn the robot at varying speeds. This allows the bot to slow down as it
        gets to its target.
        Precondition - input angle must be (-179.9,179.9) (0 or 180)
        @param angle - inputted angle that the robot will turn towards.
        @param repositioning - boolean used to toggle power so that when connected to the build plate the robot
                               can actually pull it.
    */
    public void proportionalIMU(int angle, boolean repositioning)
    {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Starting Heading: ", currentAngle());
        telemetry.addData("Heading input: ", angle);

        double left, right;

        if(angle < 179.9 && angle > 0) // (0,179.9)
        {
            while(currentAngle() > angle + .17 || currentAngle() < angle - .17) {
                double angleDifference = angle - currentAngle();

                left = angleDifference * Kp / 100;
                right = angleDifference * Kp / 100;

                if (repositioning) {
                    left = limit(left, .25, 5);
                    right = limit(right, .25, .5);

                    telemetry.addData("LEft", left);
                    telemetry.addData("Right", -right);
                    telemetry.update();

                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(-right);
                    BR.setPower(-right);
                } else {
                    left = limit(left, .12 , .4);
                    right = limit(right, .12, .4);

                    telemetry.addData("LEft", left);
                    telemetry.addData("Right", -right);
                    telemetry.update();

                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(-right);
                    BR.setPower(-right);
                }
            }
            if(!repositioning) {
                while (currentAngle() > angle + .17 || currentAngle() < angle - .17) {
                    double angleDifference = angle - currentAngle();

                    left = angleDifference * Kp / 100;
                    right = angleDifference * Kp / 100;


                        left = limit(left, .12, .5);
                        right = limit(right, .12, .5);

                        telemetry.addData("LEft", left);
                        telemetry.addData("Right", -right);
                        telemetry.update();

                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(-right);
                        BR.setPower(-right);

                }
            }

        }
        else if(angle < 0 && angle > -179.9) // (-179.9, 0)
        {
            while (currentAngle() > angle + .15 || currentAngle() < angle - .15) // (ang - .15 < current < ang + .15)
            {
                double relativeDifference;
                relativeDifference = angle + (360 - currentAngle());

                if (relativeDifference < -180){
                    relativeDifference += 360;
                } else if (relativeDifference > 180){
                    relativeDifference -= 360;
                }


                left = relativeDifference * Kp / 100;
                right = relativeDifference * Kp / 100;

                if(repositioning) {
                    left = limit(left, .23, .55);
                    right = limit(right, .23, .55);

                    telemetry.addData("Left", left);
                    telemetry.addData("Right", -right);
                    telemetry.update();

                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(-right);
                    BR.setPower(-right);
                }
                else {
                    left = limit(left, .12, .5);
                    right = limit(right, .12, .5);

                    telemetry.addData("Left", left);
                    telemetry.addData("Right", -right);
                    telemetry.update();

                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(-right);
                    BR.setPower(-right);

                }

            }

        }
        else if( angle == 180)
        {
            while(currentAngle() < 179.5) {

                double angleDifference = angleDifference(angle);

                left = angleDifference * Kp / 100;
                right = angleDifference * Kp / 100;

                if(repositioning)
                {
                    left = limit(left, .2,.5);
                    right = limit(right,.2,.5);

                    telemetry.addData("Left", -left);
                    telemetry.addData("Right", right);
                    telemetry.update();


                    FL.setPower(-left);
                    BL.setPower(-left);
                    FR.setPower(right);
                    BR.setPower(right);
                }
                else
                {
                    left = limit(left, .13, .4);
                    right = limit(right, .13, .4);

                    telemetry.addData("Left", left);
                    telemetry.addData("Right", -right);
                    telemetry.update();
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(-right);
                    BR.setPower(-right);
                }


            }

        }

        else if ( angle == 0)
        {
            while(currentAngle() < -.2 || currentAngle() > .2 ) {
                double angleDifference = angle - currentAngle();

                left = angleDifference * Kp / 100;
                right = angleDifference * Kp / 100;

                if(repositioning)
                {
                    left = limit(left, .2,.5);
                    right = limit(right, .2, .5);

                    telemetry.addData("Left", left);
                    telemetry.addData("Right", -right);
                    telemetry.update();
                }
                else
                {
                    left = limit(left, .12, .5);
                    right = limit(right, .12, .5);

                    telemetry.addData("Left", left);
                    telemetry.addData("Right", -right);
                    telemetry.update();
                }
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(-right);
                BR.setPower(-right);
            }
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final Heading", currentAngle());

    }

    /*
        sends a motor power to the drivetrain that is to be manipulated and sent out there
        no cap this is either mad nice and short or mad trash lol.
     */
    double error;
    int integral;
    float refreshConstant = .02f;
    public double PISend(int angle)
    {
        error = angle - currentAngle();
        error = error * Kp / 100;
        return error;
    }




    /*  Limit method used to ensure that an inputted value is between max and mininum inputted values.
        This method is designed to work for motor powers, if positives are entered with a negative value
        it's all good.
        @param input    - the value to be changed if necessary
        @param min      - minimum value to be returned if the input is below
        @param max      - maximum value to be returned if the input is above
     */
    public double limit(double input, double min, double max) {
        if (input > 0) {
            input += min;
            if (input > max) {
                input = max;
            }
        } else {
            input -= min;
            if (input > -min) {
                input = -min;
            }
        }
        return input;
    }

    public double  angleDifference(int angle)
    {
        if(angle >= 0 && angle != 180)
        {
            return angle - currentAngle();
        }
        else if(angle == 180)
        {
            double result = angle - (360 - Math.abs(currentAngle()));
            return result;
        }
        else
        {
            double result = Math.abs(currentAngle()) - Math.abs(angle);
            return result;
        }
    }


    /**
     * Resets the measurments so that they are starting from 0, useful if you only want to turn relative to current location,
     * rather than having to remeber the original location of the robot.
     */
    public void resetAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Checks which direction to move in,
     */
    public void turnToAngle(int angle, double power) {

        //          0
        //  90              -90
        //         180

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAngle();

        telemetry.addData("After Reset: ", currentRelativeAngle());
        telemetry.addData("Heading input: ", angle);

        double left, right;

        //added a special to check if the entered angle is 180,
        //because this will be the most used angle, so it must work best

        if(angle == 180 || angle == 0)
        {
            left =  power;
            right = -power;
            FL.setPower(left);
            BL.setPower(left);
            FR.setPower(right);
            BR.setPower(right);
            while (angle > currentRelativeAngle() && currentRelativeAngle() >= 0) {}
            if(currentRelativeAngle() < 0)
            {
                left =  -.2;
                right = .2;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < 0) {}
            }
        } else {

            //Turns based on angle set at desired power while currentAngle is not matching the target angle.
            if (angle < 0) {
                left =  -power;
                right = power;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < angle) {}

                if(currentRelativeAngle() <= angle-.5)
                {
                    left =  .15;
                    right = -.15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle) {}
                }
                // FOR SOME UNKNOWN REASON, we need to check the angle after turning twice,
                // to end the 1/7 chance of executing the check that was observed before.
                if(currentRelativeAngle() <= angle-.5)
                {
                    left =  .15;
                    right = -.15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle) {}
                }
                if(currentRelativeAngle() >= angle+1)
                {
                    left = -.12;
                    right = .12;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
                }
                if(currentRelativeAngle() >= angle+1)
                {
                    left =  -.12;
                    right = .12;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
                }
            }
            else if (angle > 0) {
                left =  power;
                right = -power;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (angle > currentRelativeAngle()) {}

                if(currentRelativeAngle() >= angle+1)
                {
                    left =  -.15;
                    right = .15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
                }
                // FOR SOME UNKNOWN REASON, we need to check the angle after turning twice,
                // to end the 1/7 chance of executing the check that was observed before.
                if(currentRelativeAngle() >= angle+1)
                {
                    left =  -.15;
                    right = .15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() > angle+.5) {}
                }


                if(currentRelativeAngle() <= angle-1)
                {
                    left =  .12;
                    right = -.12;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle-.5) {}
                }
                if(currentRelativeAngle() <= angle-1)
                {
                    left =  .15;
                    right = -.15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentRelativeAngle() < angle-.5) {}
                }

            }
            else {
                return;
            }
        }

        //Set power to 0 after the turning
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final Heading", currentRelativeAngle());

        //and another round of checks:
        if(angle > 0) {
            if(currentRelativeAngle() >= angle+1)
            {
                left =  -.15;
                right = .15;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() > angle+.5) {}
            }

            if(currentRelativeAngle() <= angle)
            {
                left =  .125;
                right = -.125;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < angle-.5) {}
            }
        }
        else{

            if(currentRelativeAngle() >= angle+1)
            {
                left =  -.15;
                right = .15;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() > angle+.5) {}
            }
            if(currentRelativeAngle() <= angle-1)
            {
                left =  .12;
                right = -.12;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentRelativeAngle() < angle-.5) {}
            }

        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final FINAL Heading", currentRelativeAngle());

        resetAngle();

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Gets a new orientation, finds the change in angle from the older orientation.
     * @return Returns the change in angle from the starting point of when the angles were last reset (During reset it is set to 0)
     */
    public double currentRelativeAngle() {
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - angles.firstAngle;

        if (changeInAngle < -180){
            changeInAngle += 360;
        } else if (changeInAngle > 180){
            changeInAngle -= 360;
        }

        return changeInAngle;
    }

    public double currentAngle() {
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double changeInAngle = newAngles.firstAngle - originalAngles.firstAngle;

        if (changeInAngle < -180){
            changeInAngle += 360;
        } else if (changeInAngle > 180){
            changeInAngle -= 360;
        }

        return changeInAngle;
    }

    public void turnOriginalAngle(int angle, double power) {

        //          0
        //  90              -90
        //     179.9   -179.9

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAngle();

        telemetry.addData("Starting: ", currentAngle());
        telemetry.addData("Heading input: ", angle);

        double left, right;

        //added a special to check if the entered angle is 180,
        //because this will be the most used angle, so it must work best

        if(angle == 180)
        {
            left =  power;
            right = -power;
            FL.setPower(left);
            BL.setPower(left);
            FR.setPower(right);
            BR.setPower(right);
            while (currentAngle() < 178 || currentAngle() == 0 ) {}
            if(currentAngle() < 0)
            {
                left =  -.2;
                right = .2;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentAngle() < 0) {}
            }
            if(currentAngle() < 179)
            {
                left =  .2;
                right = -.2;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentAngle() < 179) {}
            }
            telemetry.addData("IN HERE", currentAngle());


        }
        else {

            //Turns based on angle set at desired power while currentAngle is not matching the target angle.
            if (angle < 0) {
                if(currentAngle() > 5 || currentAngle() < -5)
                {
                    left = power;
                    right = -power;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentAngle() < angle) {
                    }
                    if (currentAngle() > angle + .5) {
                        left = -.15;
                        right = .15;
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(right);
                        BR.setPower(right);
                        while (currentAngle() > angle) {
                        }
                    }
                    if (currentAngle() > angle + .5) {
                        left = -.15;
                        right = .15;
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(right);
                        BR.setPower(right);
                        while (currentAngle() > angle) {
                        }
                    }
                    if (currentAngle() < angle - .5) {
                        left = .15;
                        right =- .15;
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(right);
                        BR.setPower(right);
                        while (currentAngle() < angle) {
                        }
                    }

                }
                else {
                    left = -power;
                    right = power;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentAngle() < angle) {
                    }

                    if (currentAngle() <= angle - .5) {
                        left = .15;
                        right = -.15;
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(right);
                        BR.setPower(right);
                        while (currentAngle() < angle) {
                        }
                    }
                    // FOR SOME UNKNOWN REASON, we need to check the angle after turning twice,
                    // to end the 1/7 chance of executing the check that was observed before.
                    if (currentAngle() <= angle - .5) {
                        left = .15;
                        right = -.15;
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(right);
                        BR.setPower(right);
                        while (currentAngle() < angle) {
                        }
                    }
                    if (currentAngle() >= angle + 1) {
                        left = -.12;
                        right = .12;
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(right);
                        BR.setPower(right);
                        while (currentAngle() > angle + .5) {
                        }
                    }
                    if (currentAngle() >= angle + 1) {
                        left = -.12;
                        right = .12;
                        FL.setPower(left);
                        BL.setPower(left);
                        FR.setPower(right);
                        BR.setPower(right);
                        while (currentAngle() > angle + .5) {
                        }
                    }
                }
            }
            else if (angle > 0) {
                left =  power;
                right = -power;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (angle > currentAngle()) {}

                if(currentAngle() >= angle+1)
                {
                    left =  -.15;
                    right = .15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentAngle() > angle+.5) {}
                }
                // FOR SOME UNKNOWN REASON, we need to check the angle after turning twice,
                // to end the 1/7 chance of executing the check that was observed before.
                if(currentAngle() >= angle+1)
                {
                    left =  -.15;
                    right = .15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentAngle() > angle+.5) {}
                }
                if(currentAngle() <= angle-1)
                {
                    left =  .12;
                    right = -.12;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentAngle() < angle-.5) {}
                }
                if(currentAngle() <= angle-1)
                {
                    left =  .15;
                    right = -.15;
                    FL.setPower(left);
                    BL.setPower(left);
                    FR.setPower(right);
                    BR.setPower(right);
                    while (currentAngle() < angle-.5) {}
                }

            }
            else {
                return;
            }
        }


        telemetry.addData("Final Heading", currentAngle());



        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final FINAL Heading", currentAngle());

        resetAngle();

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void turn90 (int direction)
    {
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAngle();

        double left, right;

        if(direction > 0)
        {
            driveTrain.spin(770,.6);

            telemetry.addData("Heading after spin" , currentAngle());

            if(currentAngle() > 90)
            {
                left =  -.2;
                right = .2;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentAngle() > 90.2) {}
            }
            sleep(100);
            if(currentAngle() < 89)
            {
                left = .2;
                right = -.2;
                FL.setPower(left);
                BL.setPower(left);
                FR.setPower(right);
                BR.setPower(right);
                while (currentAngle() < 89) {}
            }

        }
        else
        {

        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        telemetry.addData("Final FINAL Heading", currentAngle());
        telemetry.update();

        resetAngle();

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

}
