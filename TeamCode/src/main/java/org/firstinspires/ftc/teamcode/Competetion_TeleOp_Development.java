package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Competetion TeleOp Development", group="Pushbot")
//@Disabled
public class Competetion_TeleOp_Development extends LinearOpMode {

    /* Declare OpMode members. */
    Comp_Hardware robot           = new Comp_Hardware();   // Use a Pushbot's hardware
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
        double LiftMotor;
        double Motor;
        double Liftmax;
        double RelicMotor;
        double RelicLift = 0;

        robot.RelicLift =  hardwareMap.crservo.get("RL");

        /*
        RJS = hardwareMap.servo.get("RJS");
        LJS = hardwareMap.servo.get("LJS");

        GTS = hardwareMap.servo.get("GTS");
        OGTS =hardwareMap.servo.get("OGTS");
        */

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

            LiftMotor = gamepad2.left_stick_y;

            RelicMotor = gamepad1.right_stick_y;

            Liftmax = Math.max(Math.abs(LiftMotor), Math.abs(RelicMotor));
            if (Liftmax > 1.0)
            {
                LiftMotor /= Liftmax;
                RelicMotor /= Liftmax;
            }

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


            if (gamepad2.a){            //Middle pos
                robot.TLC.setPosition(0.675);
                robot.BLC.setPosition(0.52);
                robot.TRC.setPosition(0.28);
                robot.BRC.setPosition(0.8);
            }
            if (gamepad2.b){            //OPEN pos
                robot.TLC.setPosition(0.22);
                robot.BLC.setPosition(1);
                robot.TRC.setPosition(0.72);
                robot.BRC.setPosition(0.4);
            }
            if (gamepad2.x){            //CLOSE pos
                robot.TLC.setPosition(0.775);
                robot.BLC.setPosition(0.43);
                robot.TRC.setPosition(0.18);
                robot.BRC.setPosition(0.945);
            }
            if (gamepad2.y){            //Other Mid pos
                robot.TLC.setPosition(0.625);
                robot.BLC.setPosition(0.62);
                robot.TRC.setPosition(0.38);
                robot.BRC.setPosition(0.7);
            }

            if (gamepad1.x){
                robot.RelicGrab.setPosition(1);
            }
            if (gamepad1.a){
                robot.RelicGrab.setPosition(0.5);
            }

            if (gamepad2.right_bumper){
                RelicLift = gamepad2.right_stick_y;
            }





            /*
            if (gamepad1.a){
                robot.GTS.setPosition(1);
            }
            if (gamepad1.b){
                robot.GTS.setPosition(0.5);
            }

            if (gamepad1.x){
                OGTS.setPosition(0.1);
            }
            else if (gamepad1.b){
                OGTS.setPosition(0.9);
            }
            else if (gamepad1.a){
                OGTS.setPosition(0.5);
            }
            */


            // Output the safe vales to the motor drives.
            robot.FL.setPower(FL);
            robot.FR.setPower(FR);
            robot.BL.setPower(BL);
            robot.BR.setPower(BR);

            robot.LiftMotor.setPower(LiftMotor);

            robot.RelicMotor.setPower(RelicMotor);
            
            robot.RelicLift.setPower(RelicLift);

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", FL);
            telemetry.addData("right", "%.2f", FR);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
