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

@Autonomous(name="B2", group="Competition Auto")
//@Disabled
public class B2 extends LinearOpMode {

    //Adding pointless comment
    //Jewel slapper servos
    Servo LJS;

    //Clamp servos
    Servo TLC, TRC, BLC, BRC;

    //Drive motors
    DcMotor FL, FR, BL, BR;

    //Lift motor
    DcMotor LM;

    /* Declare OpMode members. */
    Comp_Hardware         robot   = new Comp_Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 392 ;      // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.116025 ; // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;      // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     DRIVE_SPEED             = 0.3;
    static final double     SLOW_DRIVE_SPEED        = 0.1;

    static final double     TURN_SPEED              = 0.5;

    static final double     LJS_down                = 0.745;
    static final double     LJS_up                  = 0.12;


    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    ColorSensor JSCL;
    DistanceSensor JSDL;

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


        //Reference the hardware devices for the code

        //DRIVE MOTORS
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        //LIFT MOTOR
        LM = hardwareMap.dcMotor.get("LM");

        //CLAMP SERVOS
        TLC = hardwareMap.servo.get("TLC");
        TRC = hardwareMap.servo.get("TRC");
        BLC = hardwareMap.servo.get("BLC");
        BRC = hardwareMap.servo.get("BRC");

        //JEWEL SLAPPER SERVOS
        LJS = hardwareMap.servo.get("LJS");

        // get a reference to the color sensor and range sensor with same name.
        JSCL = hardwareMap.get(ColorSensor.class, "CDSL");
        JSDL = hardwareMap.get(DistanceSensor.class, "CDSL");


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

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "Left");
                telemetry.update();

                ClampandLift(0.25, 1000);

                LJS.setPosition(LJS_down);
                sleep(250);
                telemetry.addData("Blue", JSCL.blue());
                telemetry.addData("Red", JSCL.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = JSCL.blue() > JSCL.red() && JSCL.blue() > JSCL.green();
                boolean RED = JSCL.red() > JSCL.blue() && JSCL.red() > JSCL.green();
                sleep(250);

                if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, -3.5, -3.5, 10);
                    LJS.setPosition(LJS_up);
                    encoderDrive(0.1, 3.5, 3.5, 10);
                }
                else if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, 3.5, 3.5, 10);
                    LJS.setPosition(LJS_up);
                    encoderDrive(0.1, -3.5, -3.5, 10);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(0.2, -42, -42, 10);
                Turn(0.2, 30);
                encoderDrive(0.2, -8, -8, 10);
                Turn(0.2, -29);
                encoderDrive(0.2, -10, -10, 10);
                Lift(-0.25, 775);
                OpenClamps();
                encoderDrive(0.1, -8, -8, 10);
                encoderDrive(0.1, 2, 2, 10);
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "Right");
                telemetry.update();

                ClampandLift(0.25, 1000);

                LJS.setPosition(LJS_down);
                sleep(250);
                telemetry.addData("Blue", JSCL.blue());
                telemetry.addData("Red", JSCL.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = JSCL.blue() > JSCL.red() && JSCL.blue() > JSCL.green();
                boolean RED = JSCL.red() > JSCL.blue() && JSCL.red() > JSCL.green();
                sleep(250);

                if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, -3.5, -3.5, 10);
                    LJS.setPosition(LJS_up);
                    encoderDrive(0.1, 3.5, 3.5, 10);
                }
                else if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, 3.5, 3.5, 10);
                    LJS.setPosition(LJS_up);
                    encoderDrive(0.1, -3.5, -3.5, 10);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(0.2, -42, -42, 10);
                Turn(0.2, 30);
                encoderDrive(0.2, -37, -37, 10);
                Turn(0.2, -28);
                encoderDrive(0.2, -8, -8, 10);
                Lift(-0.25, 775);
                OpenClamps();
                encoderDrive(0.1, -8, -8, 10);
                encoderDrive(0.1, 2, 2, 10);
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("VuMark", "Center");
                telemetry.update();

                ClampandLift(0.25, 1000);

                LJS.setPosition(LJS_down);
                sleep(250);
                telemetry.addData("Blue", JSCL.blue());
                telemetry.addData("Red", JSCL.red());
                telemetry.update();
                sleep(250);
                boolean BLUE = JSCL.blue() > JSCL.red() && JSCL.blue() > JSCL.green();
                boolean RED = JSCL.red() > JSCL.blue() && JSCL.red() > JSCL.green();
                sleep(250);

                if (RED) {
                    telemetry.addData("Red", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, -3.5, -3.5, 10);
                    LJS.setPosition(LJS_up);
                    encoderDrive(0.1, 3.5, 3.5, 10);
                }
                else if (BLUE) {
                    telemetry.addData("Blue", "Seen");
                    telemetry.update();
                    encoderDrive(0.1, 3.5, 3.5, 10);
                    LJS.setPosition(LJS_up);
                    encoderDrive(0.1, -3.5, -3.5, 10);
                }
                else {
                    telemetry.addData("Thou Hath Recieveth", "L");
                    telemetry.update();
                }

                sleep(250);

                encoderDrive(0.2, -42, -42, 10);
                Turn(0.2, 30);
                encoderDrive(0.2, -22, -22, 10);
                Turn(0.2, -28);
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

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = robot.FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFRTarget = robot.FR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBLTarget = robot.BL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBRTarget = robot.BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.FL.setTargetPosition(newFLTarget);
            robot.FR.setTargetPosition(newFRTarget);
            robot.BL.setTargetPosition(newBLTarget);
            robot.BR.setTargetPosition(newBRTarget);

            // Turn On RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.FL.setPower(speed);
            robot.FR.setPower(speed);
            robot.BL.setPower(speed);
            robot.BR.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FL.isBusy() && robot.FR.isBusy() && robot.BL.isBusy() && robot.BR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.FL.getCurrentPosition(),
                        robot.FR.getCurrentPosition(),
                        robot.BL.getCurrentPosition(),
                        robot.BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    //----------------------------------------------------------------------------------------------

    public void Turn (double speed, double degrees) {


        int newFleftTarget;
        int newFrightTarget;
        int newBleftTarget;
        int newBrightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFleftTarget = robot.FL.getCurrentPosition() - (int)(degrees * COUNTS_PER_INCH);
            newFrightTarget = robot.FR.getCurrentPosition() + (int)(degrees * COUNTS_PER_INCH);
            newBleftTarget = robot.BL.getCurrentPosition() - (int)(degrees * COUNTS_PER_INCH);
            newBrightTarget = robot.BR.getCurrentPosition() + (int)(degrees * COUNTS_PER_INCH);

            robot.FL.setTargetPosition(newFleftTarget);
            robot.FR.setTargetPosition(newFrightTarget);
            robot.BL.setTargetPosition(newBleftTarget);
            robot.BR.setTargetPosition(newBrightTarget);

            // Turn On RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FL.setPower(-speed);
            robot.FR.setPower(speed);
            robot.BL.setPower(-speed);
            robot.BR.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.FL.isBusy() && robot.FR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFleftTarget,  newFrightTarget, newBleftTarget, newBrightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.FL.getCurrentPosition(),
                        robot.FR.getCurrentPosition(),
                        robot.BL.getCurrentPosition(),
                        robot.BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);

            //Reset Encoders
            robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }

    //----------------------------------------------------------------------------------------------

    public void Shift (double speed,
                       double Inches,
                       double timeoutS) {

        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = robot.FL.getCurrentPosition() - (int)(Inches * COUNTS_PER_INCH);
            newFRTarget = robot.FR.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            newBLTarget = robot.BL.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            newBRTarget = robot.BR.getCurrentPosition() - (int)(Inches * COUNTS_PER_INCH);

            robot.FL.setTargetPosition(newFLTarget);
            robot.FR.setTargetPosition(newFRTarget);
            robot.BL.setTargetPosition(newBLTarget);
            robot.BR.setTargetPosition(newBRTarget);

            // Turn On RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FL.setPower(-speed);
            robot.FR.setPower(speed);
            robot.BL.setPower(speed);
            robot.BR.setPower(-speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FL.isBusy() && robot.FR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.FL.getCurrentPosition(),
                        robot.FR.getCurrentPosition(),
                        robot.BL.getCurrentPosition(),
                        robot.BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);

            // Reset Encoders
            robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }

    //----------------------------------------------------------------------------------------------

    public void Lift(double LiftSpeed, long TimeofLift) {
        robot.LiftMotor.setPower(-LiftSpeed);
        sleep(TimeofLift);
        robot.LiftMotor.setPower(0);
    }

    //----------------------------------------------------------------------------------------------

    public void ClampandLift(double LiftSpeed, long TimeofLift) {
        CloseClamps();
        sleep(150);
        robot.LiftMotor.setPower(-LiftSpeed);
        sleep(TimeofLift);
        robot.LiftMotor.setPower(0);
    }

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

    public void OpenClamps() {
        robot.TLC.setPosition(0.22);
        robot.BLC.setPosition(1);
        robot.TRC.setPosition(0.72);
        robot.BRC.setPosition(0.4);
    }

    //----------------------------------------------------------------------------------------------

    public void CloseClamps() {
        robot.TLC.setPosition(0.775);
        robot.BLC.setPosition(0.43);
        robot.TRC.setPosition(0.18);
        robot.BRC.setPosition(0.945);
    }

    //----------------------------------------------------------------------------------------------

    public void MidClamps() {
        TLC.setPosition(0.95);
        TRC.setPosition(0.45);
        BLC.setPosition(0.05);
        BRC.setPosition(0.6);
    }

    //----------------------------------------------------------------------------------------------

    public void Wait(long Seconds) {
        sleep(Seconds * 1000);
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
