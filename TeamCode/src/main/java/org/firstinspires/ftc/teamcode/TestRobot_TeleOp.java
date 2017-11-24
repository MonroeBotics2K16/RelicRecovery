package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test Robot TeleOp", group="Pushbot")
//@Disabled
public class TestRobot_TeleOp extends LinearOpMode {

    public Servo GTS, OGTS;

    /* Declare OpMode members. */
    HardwareTestRobot robot           = new HardwareTestRobot();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() {
        double FL;
        double FR;
        double BL;
        double BR;
        double drive;
        double turn;
        double max;


        GTS = hardwareMap.servo.get("GTS");
        OGTS = hardwareMap.servo.get("OGTS");

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Combine drive and turn for blended motion.
            FL  = gamepad1.left_stick_y - gamepad1.left_stick_x;
            FR  = gamepad1.left_stick_y + gamepad1.left_stick_x;
            BL  = gamepad1.left_stick_y - gamepad1.left_stick_x;
            BR  = gamepad1.left_stick_y + gamepad1.left_stick_x;



            if (!gamepad1.right_bumper) {
                FL  = gamepad1.left_stick_y - gamepad1.left_stick_x;
                FR  = gamepad1.left_stick_y + gamepad1.left_stick_x;
                BL  = gamepad1.left_stick_y - gamepad1.left_stick_x;
                BR  = gamepad1.left_stick_y + gamepad1.left_stick_x;

                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(FL), Math.abs(FR));
                if (max > 1.0)
                {
                    FL /= max;
                    FR /= max;
                }
                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(BL), Math.abs(BR));
                if (max > 1.0)
                {
                    BL /= max;
                    BR /= max;
                }
            }
            if (gamepad1.right_bumper) {
                FL = -gamepad1.right_stick_x;
                FR = gamepad1.right_stick_x;
                BL = gamepad1.right_stick_x;
                BR = -gamepad1.right_stick_x;

                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(FL), Math.abs(FR));
                if (max > 1.0)
                {
                    FL /= max;
                    FR /= max;
                }
                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(BL), Math.abs(BR));
                if (max > 1.0)
                {
                    BL /= max;
                    BR /= max;
                }
            }
            if (gamepad1.left_bumper) {
                FL = -gamepad1.left_stick_y;
                BR = -gamepad1.left_stick_y;
                FR = gamepad1.right_stick_y;
                BL = gamepad1.right_stick_y;
            }

            if (gamepad1.a){
                GTS.setPosition(1);
            }
            if (gamepad1.b){
                GTS.setPosition(0.5);
            }

            if (gamepad1.x){
                OGTS.setPosition(1);
            }
            if (gamepad1.y){
                OGTS.setPosition(0);
            }

            // Output the safe vales to the motor drives.
            robot.FL.setPower(FL);
            robot.FR.setPower(FR);
            robot.BL.setPower(BL);
            robot.BR.setPower(BR);



            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
