package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Comp_Hardware
{
    /* Public OpMode members. */
    public DcMotor  flMotor  = null;
    public DcMotor  frMotor  = null;
    public DcMotor  blMotor  = null;
    public DcMotor  brMotor  = null;

    public DcMotor  liftMotor = null;

    public DcMotor  relicMotor  = null;

    public Servo    tlClamp  = null;
    public Servo    trClamp  = null;
    public Servo    brClamp  = null;
    public Servo    blClamp  = null;

    public Servo    lJewelServo  = null;
    public Servo    rJewelServo  = null;

    public Servo    relicGate = null;
    public Servo    relicGrab = null;
    public Servo    relicLift = null;

    public final double LJS_UP = 0.12;
    public final double RJS_UP = 1;

    ColorSensor leftColorSensor;
    DistanceSensor leftDistanceSensor;
    
    DigitalChannel digin;                // Lift Touch Sensor


    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Comp_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        flMotor = hwMap.get(DcMotor.class, "FL");
        frMotor = hwMap.get(DcMotor.class, "FR");
        blMotor = hwMap.get(DcMotor.class, "BL");
        brMotor = hwMap.get(DcMotor.class, "BR");

        liftMotor  = hwMap.get(DcMotor.class, "LM");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        relicMotor = hwMap.get(DcMotor.class, "RM");

        tlClamp  = hwMap.get(Servo.class, "TLC");
        trClamp  = hwMap.get(Servo.class, "TRC");
        brClamp  = hwMap.get(Servo.class, "BRC");
        blClamp  = hwMap.get(Servo.class, "BLC");

        lJewelServo = hwMap.get(Servo.class, "LJS");
        rJewelServo = hwMap.get(Servo.class, "RJS");

        relicGrab = hwMap.get(Servo.class, "RGr");
        relicLift = hwMap.get(Servo.class, "RL");
        relicGate = hwMap.get(Servo.class, "RGa");

        leftColorSensor = hwMap.get(ColorSensor.class, "CDSL");
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "CDSL");

        digin  = hwMap.get(DigitalChannel.class, "LT");     //  Use generic form of device mapping
        digin.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel


        flMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        blMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        brMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        relicMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);

        liftMotor.setPower(0);

        relicMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lJewelServo.setPosition(LJS_UP);
        rJewelServo.setPosition(RJS_UP);

        tlClamp.setPosition(0.22);
        blClamp.setPosition(1);
        trClamp.setPosition(0.72);
        brClamp.setPosition(0.4);

        relicLift.setPosition(0.7);
        relicGate.setPosition(1);
    }
 }

