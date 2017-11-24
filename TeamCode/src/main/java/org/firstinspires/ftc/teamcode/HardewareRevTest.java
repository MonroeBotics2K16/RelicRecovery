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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardewareRevTest
{
    /* Public OpMode members. */
    public DcMotor  FL  = null;
    public DcMotor  FR  = null;
    public DcMotor  BL  = null;
    public DcMotor  BR  = null;

    public DcMotor  LiftMotor = null;

    public Servo    TLC  = null;
    public Servo    TRC  = null;
    public Servo    BRC  = null;
    public Servo    BLC  = null;

    public Servo    LJS  = null;
    public Servo    RJS  = null;

    public Servo    GTS  = null;
    public Servo    OGTS = null;

    public final double LJS_UP = 0.12;
    public final double RJS_UP = 1;


    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardewareRevTest(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL = hwMap.get(DcMotor.class, "FL");
        FR = hwMap.get(DcMotor.class, "FR");
        BL = hwMap.get(DcMotor.class, "BL");
        BR = hwMap.get(DcMotor.class, "BR");

        LiftMotor  = hwMap.get(DcMotor.class, "LM");

        TLC  = hwMap.get(Servo.class, "TLC");
        TRC  = hwMap.get(Servo.class, "TRC");
        BRC  = hwMap.get(Servo.class, "BRC");
        BLC  = hwMap.get(Servo.class, "BLC");

        LJS = hwMap.get(Servo.class, "LJS");
        RJS = hwMap.get(Servo.class, "RJS");

        GTS = hwMap.get(Servo.class, "GTS");
        OGTS = hwMap.get(Servo.class, "OGTS");

        FL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        FR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        BL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        BR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        LiftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        LiftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LJS.setPosition(LJS_UP);
        RJS.setPosition(RJS_UP);

        TLC.setPosition(0.22);
        BLC.setPosition(1);
        TRC.setPosition(0.72);
        BRC.setPosition(0.4);
    }
 }

