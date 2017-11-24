/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="R2", group="Competition Auto")
//@Disabled
public class R2 extends CommonClass {

    static final double     DRIVE_SPEED             = 0.3;
    static final double     SLOW_DRIVE_SPEED        = 0.1;

    static final double     TURN_SPEED              = 0.5;

    static final double     RJS_down                = 0.36;
    static final double     RJS_up                  = 1;


    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    ColorSensor JSCR;
    DistanceSensor JSDR;

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
         * and paste it in to your code onthe next line, between the double quotes.
         */
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


        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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
         * and paste it in to your code onthe next line, between the double quotes.
         */

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

        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        relicTrackables.activate();
        composeTelemetry();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Ready to kick a$$");
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

            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "RighT");
                telemetry.update();

                ClampandLift(0.25, 1000);

                robot.RJS.setPosition(RJS_down);
                sleep(250);
                telemetry.addData("Blue", JSCR.blue());
                telemetry.addData("Red", JSCR.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = JSCR.blue() > JSCR.red() && JSCR.blue() > JSCR.green();
                boolean RED = JSCR.red() > JSCR.blue() && JSCR.red() > JSCR.green();
                sleep(250);

                if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, -3.5, -3.5, 10);
                    robot.RJS.setPosition(RJS_up);
                    encoderDrive(0.1, 3.5, 3.5, 10);
                }
                else if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, 3.5, 3.5, 10);
                    robot.RJS.setPosition(RJS_up);
                    encoderDrive(0.1, -3.5, -3.5, 10);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(0.2, -42, -42, 10);
                Turn(0.2, -30);
                encoderDrive(0.2, -8, -8, 10);
                Turn(0.2, 29);
                encoderDrive(0.2, -10, -10, 10);
                Lift(-0.25, 775);
                OpenClamps();
                encoderDrive(0.1, -8, -8, 10);
                encoderDrive(0.1, 2, 2, 10);
            }
            else if (vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "LefT");
                telemetry.update();

                ClampandLift(0.25, 1000);

                robot.RJS.setPosition(RJS_down);
                sleep(250);
                telemetry.addData("Blue", JSCR.blue());
                telemetry.addData("Red", JSCR.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = JSCR.blue() > JSCR.red() && JSCR.blue() > JSCR.green();
                boolean RED = JSCR.red() > JSCR.blue() && JSCR.red() > JSCR.green();
                sleep(250);

                if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, -3.5, -3.5, 10);
                    robot.RJS.setPosition(RJS_up);
                    encoderDrive(0.1, 3.5, 3.5, 10);
                }
                else if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, 3.5, 3.5, 10);
                    robot.RJS.setPosition(RJS_up);
                    encoderDrive(0.1, -3.5, -3.5, 10);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(0.2, -42, -42, 10);
                Turn(0.2, -30);
                encoderDrive(0.2, -37, -37, 10);
                Turn(0.2, 28);
                encoderDrive(0.2, -10, -10, 10);
                Lift(-0.25, 775);
                OpenClamps();
                encoderDrive(0.1, -6, -6, 10);
                encoderDrive(0.1, 2, 2, 10);
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("VuMark", "Center");
                telemetry.update();

                ClampandLift(0.25, 1000);

                robot.RJS.setPosition(RJS_down);
                sleep(250);
                telemetry.addData("Blue", JSCR.blue());
                telemetry.addData("Red", JSCR.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = JSCR.blue() > JSCR.red() && JSCR.blue() > JSCR.green();
                boolean RED = JSCR.red() > JSCR.blue() && JSCR.red() > JSCR.green();
                sleep(250);

                if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, -3.5, -3.5, 10);
                    robot.RJS.setPosition(RJS_up);
                    encoderDrive(0.1, 3.5, 3.5, 10);
                }
                else if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, 3.5, 3.5, 10);
                    robot.RJS.setPosition(RJS_up);
                    encoderDrive(0.1, -3.5, -3.5, 10);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(0.2, -42, -42, 10);
                Turn(0.2, -30);
                encoderDrive(0.2, -22, -22, 10);
                Turn(0.2, 28);
                encoderDrive(0.2, -10, -10, 10);
                Lift(-0.25, 775);
                OpenClamps();
                encoderDrive(0.1, -8, -8, 10);
                encoderDrive(0.1, 2, 2, 10);
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

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

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
