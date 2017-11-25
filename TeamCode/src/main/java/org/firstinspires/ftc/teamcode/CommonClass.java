package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by MonroeBotics on 11/24/2017.
 */

public class CommonClass extends LinearOpMode {

    /* Declare OpMode members. */
            Comp_Hardware         robot   = new Comp_Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 392 ;      // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.116025 ; // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;      // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {

    }



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
            newFLTarget = robot.flMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFRTarget = robot.frMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBLTarget = robot.blMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBRTarget = robot.brMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.flMotor.setTargetPosition(newFLTarget);
            robot.frMotor.setTargetPosition(newFRTarget);
            robot.blMotor.setTargetPosition(newBLTarget);
            robot.brMotor.setTargetPosition(newBRTarget);

            // Turn On RUN_TO_POSITION
            robot.flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.flMotor.setPower(speed);
            robot.frMotor.setPower(speed);
            robot.blMotor.setPower(speed);
            robot.brMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.flMotor.isBusy() && robot.frMotor.isBusy() && robot.blMotor.isBusy() && robot.brMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.flMotor.getCurrentPosition(),
                        robot.frMotor.getCurrentPosition(),
                        robot.blMotor.getCurrentPosition(),
                        robot.brMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    //----------------------------------------------------------------------------------------------

    public void encoderLift(double speed,
                            double inches,
                            double timeoutS) {
        int newLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.liftMotor.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLiftTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
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
            newFleftTarget = robot.flMotor.getCurrentPosition() - (int)(degrees * COUNTS_PER_INCH);
            newFrightTarget = robot.frMotor.getCurrentPosition() + (int)(degrees * COUNTS_PER_INCH);
            newBleftTarget = robot.blMotor.getCurrentPosition() - (int)(degrees * COUNTS_PER_INCH);
            newBrightTarget = robot.brMotor.getCurrentPosition() + (int)(degrees * COUNTS_PER_INCH);

            robot.flMotor.setTargetPosition(newFleftTarget);
            robot.frMotor.setTargetPosition(newFrightTarget);
            robot.blMotor.setTargetPosition(newBleftTarget);
            robot.brMotor.setTargetPosition(newBrightTarget);

            // Turn On RUN_TO_POSITION
            robot.flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.flMotor.setPower(-speed);
            robot.frMotor.setPower(speed);
            robot.blMotor.setPower(-speed);
            robot.brMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.flMotor.isBusy() && robot.frMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFleftTarget,  newFrightTarget, newBleftTarget, newBrightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.flMotor.getCurrentPosition(),
                        robot.frMotor.getCurrentPosition(),
                        robot.blMotor.getCurrentPosition(),
                        robot.brMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);

            //Reset Encoders
            robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }

    //----------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------

    public void Lift(double LiftSpeed, long TimeofLift) {
        robot.liftMotor.setPower(-LiftSpeed);
        sleep(TimeofLift);
        robot.liftMotor.setPower(0);
    }

    //----------------------------------------------------------------------------------------------

    public void ClampandLift(double LiftSpeed, long TimeofLift) {
        CloseClamps();
        sleep(150);
        robot.liftMotor.setPower(-LiftSpeed);
        sleep(TimeofLift);
        robot.liftMotor.setPower(0);
    }

    //----------------------------------------------------------------------------------------------

    public void OpenClamps() {
        robot.tlClamp.setPosition(0.22);
        robot.blClamp.setPosition(1);
        robot.trClamp.setPosition(0.72);
        robot.brClamp.setPosition(0.4);
    }

    //----------------------------------------------------------------------------------------------

    public void CloseClamps() {
        robot.tlClamp.setPosition(0.775);
        robot.blClamp.setPosition(0.43);
        robot.trClamp.setPosition(0.18);
        robot.brClamp.setPosition(0.945);
    }

    //----------------------------------------------------------------------------------------------

}
