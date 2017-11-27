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
        double flMotor;
        double frMotor;
        double blMotor;
        double brMotor;
        double max;
        double liftMotor = 0;
        double liftMax;
        double relicMotor;
        double relicLift = 0;

        boolean buttonStateB = false;
        boolean buttonStateA = false;
        boolean buttonStateY = false;


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
            flMotor  = gamepad1.left_stick_y - gamepad1.left_stick_x;
            frMotor  = gamepad1.left_stick_y + gamepad1.left_stick_x;
            blMotor  = gamepad1.left_stick_y - gamepad1.left_stick_x;
            brMotor  = gamepad1.left_stick_y + gamepad1.left_stick_x;


            if (!robot.liftTouch.getState() && gamepad2.left_stick_y >= 0) {
                liftMotor = 0;
            }
            else{
                liftMotor = gamepad2.left_stick_y;
            }
            
            relicMotor = gamepad1.right_stick_y;

            liftMax = Math.max(Math.abs(liftMotor), Math.abs(relicMotor));
            if (liftMax > 1.0)
            {
                liftMotor /= liftMax;
                relicMotor /= liftMax;
            }

            if (!gamepad1.right_bumper) {
                flMotor  = gamepad1.left_stick_y - gamepad1.left_stick_x;
                frMotor  = gamepad1.left_stick_y + gamepad1.left_stick_x;
                blMotor  = gamepad1.left_stick_y - gamepad1.left_stick_x;
                brMotor  = gamepad1.left_stick_y + gamepad1.left_stick_x;

                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(flMotor), Math.abs(frMotor));
                if (max > 1.0)
                {
                    flMotor /= max;
                    frMotor /= max;
                }
                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(blMotor), Math.abs(brMotor));
                if (max > 1.0)
                {
                    blMotor /= max;
                    brMotor /= max;
                }
            }
            if (gamepad1.right_bumper) {
                flMotor = -gamepad1.right_stick_x;
                frMotor = gamepad1.right_stick_x;
                blMotor = gamepad1.right_stick_x;
                brMotor = -gamepad1.right_stick_x;

                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(flMotor), Math.abs(frMotor));
                if (max > 1.0)
                {
                    flMotor /= max;
                    frMotor /= max;
                }
                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(blMotor), Math.abs(brMotor));
                if (max > 1.0)
                {
                    blMotor /= max;
                    brMotor /= max;
                }
            }
            if (gamepad1.left_bumper) {
                flMotor = -gamepad1.left_stick_y;
                brMotor = -gamepad1.left_stick_y;
                frMotor = gamepad1.right_stick_y;
                blMotor = gamepad1.right_stick_y;
            }

            //--------------------------------------------------------------------------------------

            //Middle position
            if (gamepad2.right_bumper && gamepad2.b) {
                buttonStateB = true;
            }
            if (buttonStateB && robot.liftMotor.getCurrentPosition() >= -5800){//change to target {
                liftMotor = -1;
            }
            else if (buttonStateB && robot.liftMotor.getCurrentPosition() <= -5800){
                buttonStateB = false;
                liftMotor = 0;
            }

            //--------------------------------------------------------------------------------------

            //Low position
            if (gamepad2.right_bumper && gamepad2.a) {
                buttonStateA = true;
            }
            if (buttonStateA && robot.liftMotor.getCurrentPosition() >= -3000){
                liftMotor = -1;
            }
            else if (buttonStateA && robot.liftMotor.getCurrentPosition() <=-3000){
                buttonStateA = false;
                liftMotor = 0;
            }

            //--------------------------------------------------------------------------------------

            //High postion

            if (gamepad2.right_bumper && gamepad2.y) {
                buttonStateY = true;
            }
            if (buttonStateY && robot.liftMotor.getCurrentPosition() >= -8750) {
                liftMotor = -1;
            }
            else if (buttonStateY && robot.liftMotor.getCurrentPosition() <= -8750) {
                buttonStateY = false;
                liftMotor = 0;
            }


            if (!gamepad2.right_bumper && gamepad2.a){            //Middle pos
                robot.tlClamp.setPosition(0.675);
                robot.blClamp.setPosition(0.52);
                robot.trClamp.setPosition(0.28);
                robot.brClamp.setPosition(0.8);
            }
            if (!gamepad2.right_bumper && gamepad2.b){            //OPEN pos
                robot.tlClamp.setPosition(0.22);
                robot.blClamp.setPosition(1);
                robot.trClamp.setPosition(0.72);
                robot.brClamp.setPosition(0.4);
            }
            if (!gamepad2.right_bumper && gamepad2.x){            //CLOSE pos
                robot.tlClamp.setPosition(0.775);
                robot.blClamp.setPosition(0.43);
                robot.trClamp.setPosition(0.18);
                robot.brClamp.setPosition(0.945);
            }
            if (!gamepad2.right_bumper && gamepad2.y){            //Other Mid pos
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
            robot.flMotor.setPower(flMotor);
            robot.frMotor.setPower(frMotor);
            robot.blMotor.setPower(blMotor);
            robot.brMotor.setPower(brMotor);

            robot.liftMotor.setPower(liftMotor);

            robot.relicMotor.setPower(relicMotor);


            // Send telemetry message to signify robot running;
            telemetry.addData("left", "%.2f", flMotor);
            telemetry.addData("right", "%.2f", frMotor);
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
