package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        double LiftMotor = 0;
        double Motor;
        double Liftmax;
        double RelicMotor;
        double RelicLift = 0;


        boolean buttonState = false;


        //RJS = hardwareMap.servo.get("RJS");
        //LJS = hardwareMap.servo.get("LJS");

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


            if (!robot.liftTouch.getState() && gamepad2.left_stick_y >= 0) {
                LiftMotor = 0;
            }
            else{
                LiftMotor = gamepad2.left_stick_y;
            }



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


            if (gamepad2.right_bumper && gamepad2.b) {
                buttonState = true;
            }

            if (buttonState && robot.liftMotor.getCurrentPosition() <= 20){//change to target {
                LiftMotor = -1;
            }
            else if (buttonState && robot.liftMotor.getCurrentPosition() >= 20){
                buttonState = false;
                LiftMotor = 0;
            }


            if (!gamepad2.right_bumper && gamepad2.a){            //Middle pos
                robot.tlClamp.setPosition(0.675);
                robot.blClamp.setPosition(0.52);
                robot.trClamp.setPosition(0.28);
                robot.brClamp.setPosition(0.8);
            }
            if (gamepad2.b){            //OPEN pos
                robot.tlClamp.setPosition(0.22);
                robot.blClamp.setPosition(1);
                robot.trClamp.setPosition(0.72);
                robot.brClamp.setPosition(0.4);
            }
            if (gamepad2.x){            //CLOSE pos
                robot.tlClamp.setPosition(0.775);
                robot.blClamp.setPosition(0.43);
                robot.trClamp.setPosition(0.18);
                robot.brClamp.setPosition(0.945);
            }
            if (gamepad2.y){            //Other Mid pos
                robot.tlClamp.setPosition(0.625);
                robot.blClamp.setPosition(0.62);
                robot.trClamp.setPosition(0.38);
                robot.brClamp.setPosition(0.7);
            }

            /*if (gamepad1.x){
                robot.RelicGrab.setPosition(1);
            }
            if (gamepad1.a){
                robot.RelicGrab.setPosition(0.5);
            }*/

            if (gamepad2.right_bumper){
                RelicLift = gamepad2.right_stick_y;
            }


            if (gamepad1.dpad_up){
                robot.relicGrab.setPosition(1);
            }
            if (gamepad1.dpad_down){
                robot.relicGrab.setPosition(0);
            }

            if (gamepad1.x){
                robot.relicLift.setPosition(0.615);
            }
            else if (gamepad1.b){
                robot.relicLift.setPosition(0.7);
            }
            else if (gamepad1.a){
                robot.relicLift.setPosition(0.66);
            }

            if (gamepad2.dpad_up){              //Close Gate
                robot.relicGate.setPosition(1);
            }
            else if (gamepad2.dpad_down){       //Open Gate
                robot.relicGate.setPosition(0.5);
            }

            // Output the safe vales to the motor drives.
            robot.flMotor.setPower(FL);
            robot.frMotor.setPower(FR);
            robot.blMotor.setPower(BL);
            robot.brMotor.setPower(BR);

            robot.liftMotor.setPower(LiftMotor);

            robot.relicMotor.setPower(RelicMotor);


            // Send telemetry message to signify robot running;
            telemetry.addData("left", "%.2f", FL);
            telemetry.addData("right", "%.2f", FR);
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
