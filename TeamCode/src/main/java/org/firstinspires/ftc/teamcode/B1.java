package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@Autonomous(name="B1", group="Competition Auto")
//@Disabled
public class B1 extends CommonClass {


    static final double     DRIVE_SPEED             = 0.2;
    static final double     SLOW_DRIVE_SPEED        = 0.1;

    static final double     TURN_SPEED              = 0.2;

    static final double     LJS_DOWN                = 0.745;
    static final double     LJS_UP                  = 0.12;
    
    static final double     TIMEOUT                 = 10;

    static final double     LIFT_SPEED              = 1;
    static final long       LIFT_TIME               = 1000;
    
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    public static final String TAG = "Vuforia VuMark Test";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters Parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        
        //License Key
        Parameters.vuforiaLicenseKey = "AZ9bf+//////AAAAGUN72AqPRkmNgnmzA1uEA5gcX4C+jBFFcFfvzH44crjQ/y2qPZEhKrpPs/Swo9Dt+5mVudOkuZmyGAGczv/WWPxhvQncLrqXZMBv5B6aNWi6w1Nr69ixM9lNMmaK88aUUQ17oz8LTXUPyjqulEvRcsoD/+LYrllx3/5TxlucQYKNMdF1xN4Jdp9TVeFk+jHMBLb0EKcc+vThfxW4RyRTimk2xX3xmhz00sewFoX+nb4ydLyBPXjovayEJBYSdavGYuKABz0YdbGa4XBoPMLf5gQlAb5wBYebR2+Pn/Fs72cmeieUP69UulbjSeNcfjwh4m+UhSbhGjBCkn1leCxjUJgE4ldDpIwff+saKq/H0J9Q";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        Parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(Parameters);


        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        robot.init(hardwareMap);


        //------------------------------------------------------------------------------------------
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        //------------------------------------------------------------------------------------------
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        relicTrackables.activate();
        composeTelemetry();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Ready for Battle Sir");
        telemetry.update();
        waitForStart();
        //------------------------------------------------------------------------------------------



        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {


            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();

                ClampandLift(LIFT_SPEED, LIFT_TIME);

                robot.lJewelServo.setPosition(LJS_DOWN);
                sleep(250);
                telemetry.addData("Blue", robot.JSCL.blue());
                telemetry.addData("Red", robot.JSCL.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = robot.JSCL.blue() > robot.JSCL.red() && robot.JSCL.blue() > robot.JSCL.green();
                boolean RED = robot.JSCL.red() > robot.JSCL.blue() && robot.JSCL.red() > robot.JSCL.green();
                sleep(250);

                if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(SLOW_DRIVE_SPEED, 3.5, 3.5, TIMEOUT);
                    robot.lJewelServo.setPosition(LJS_UP);
                    encoderDrive(SLOW_DRIVE_SPEED, -3.5, -3.5, TIMEOUT);
                }
                else if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(SLOW_DRIVE_SPEED, -3.5, -3.5, TIMEOUT);
                    robot.lJewelServo.setPosition(LJS_UP);
                    encoderDrive(SLOW_DRIVE_SPEED, 3.5, 3.5, TIMEOUT);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(DRIVE_SPEED, -54, -54, TIMEOUT);
                Turn(TURN_SPEED, -33.5);
                encoderDrive(DRIVE_SPEED, -5, -5, 5);
                Lift(-0.25, 800);
                OpenClamps();
                encoderDrive(SLOW_DRIVE_SPEED, -5, -5, TIMEOUT);
                encoderDrive(SLOW_DRIVE_SPEED, 2, 2, TIMEOUT);
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "Right");
                telemetry.update();

                ClampandLift(LIFT_SPEED, LIFT_TIME);

                robot.lJewelServo.setPosition(LJS_DOWN);
                sleep(250);
                telemetry.addData("Blue", robot.JSCL.blue());
                telemetry.addData("Red", robot.JSCL.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = robot.JSCL.blue() > robot.JSCL.red() && robot.JSCL.blue() > robot.JSCL.green();
                boolean RED = robot.JSCL.red() > robot.JSCL.blue() && robot.JSCL.red() > robot.JSCL.green();
                sleep(250);

                if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(SLOW_DRIVE_SPEED, 3.5, 3.5, TIMEOUT);
                    robot.lJewelServo.setPosition(LJS_UP);
                    encoderDrive(SLOW_DRIVE_SPEED, -3.5, -3.5, TIMEOUT);
                }
                else if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(SLOW_DRIVE_SPEED, -3.5, -3.5, TIMEOUT);
                    robot.lJewelServo.setPosition(LJS_UP);
                    encoderDrive(SLOW_DRIVE_SPEED, 3.5, 3.5, TIMEOUT);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(DRIVE_SPEED, -82, -82, TIMEOUT);
                Turn(TURN_SPEED, -33.5);
                encoderDrive(DRIVE_SPEED, -5, -5, 5);
                Lift(-0.25, 800);
                OpenClamps();
                encoderDrive(SLOW_DRIVE_SPEED, -5, -5, TIMEOUT);
                encoderDrive(SLOW_DRIVE_SPEED, 2, 2, TIMEOUT);
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("VuMark", "Center");
                telemetry.update();

                ClampandLift(LIFT_SPEED, LIFT_TIME);

                robot.lJewelServo.setPosition(LJS_DOWN);
                sleep(250);
                telemetry.addData("Blue", robot.JSCL.blue());
                telemetry.addData("Red", robot.JSCL.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = robot.JSCL.blue() > robot.JSCL.red() && robot.JSCL.blue() > robot.JSCL.green();
                boolean RED = robot.JSCL.red() > robot.JSCL.blue() && robot.JSCL.red() > robot.JSCL.green();
                sleep(250);

                if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(SLOW_DRIVE_SPEED, 3.5, 3.5, TIMEOUT);
                    robot.lJewelServo.setPosition(LJS_UP);
                    encoderDrive(SLOW_DRIVE_SPEED, -3.5, -3.5, TIMEOUT);
                }
                else if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(SLOW_DRIVE_SPEED, -3.5, -3.5, TIMEOUT);
                    robot.lJewelServo.setPosition(LJS_UP);
                    encoderDrive(SLOW_DRIVE_SPEED, 3.5, 3.5, TIMEOUT);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(DRIVE_SPEED, -68, -68, TIMEOUT);
                Turn(TURN_SPEED, -33.5);
                encoderDrive(DRIVE_SPEED, -5, -5, 5);
                Lift(-0.25, 800);
                OpenClamps();
                encoderDrive(SLOW_DRIVE_SPEED, -5, -5, TIMEOUT);
                encoderDrive(SLOW_DRIVE_SPEED, 2, 2, TIMEOUT);
            }

            else {
                telemetry.addData("VuMark", "Not Seen");
                telemetry.update();
            }

            while (opModeIsActive()) {
                telemetry.update();
            }

        }

    }

    //**********************************************************************************************
    //THESE LINES SEPARATE CODE FROM METHODS
    //**********************************************************************************************

    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------

    //**********************************************************************************************
    //FORMATTING
    //**********************************************************************************************

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //----------------------------------------------------------------------------------------------
}
