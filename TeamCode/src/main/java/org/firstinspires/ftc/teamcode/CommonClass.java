package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

            //  sleep(250);   // optional pause after each move
        }
    }

}
