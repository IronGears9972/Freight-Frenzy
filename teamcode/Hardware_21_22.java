package org.firstinspires.ftc.teamcode.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware_21_22 {

    //Give names to our Motors for our Programs
    public DcMotorEx frontLeftMotor = null;
    public DcMotorEx frontRightMotor = null;
    public DcMotorEx rearLeftMotor = null;
    public DcMotorEx rearRightMotor = null;
    public DcMotor intakemotor = null;
    public DcMotor lifter = null;
    public DcMotor duckextend = null;

    //Give names to our Servos for our Programs
    public Servo elementarm = null;
    public Servo lightsaber = null;
    public Servo elementclamp1 = null;
    public Servo elementclamp2 = null;
    public CRServo duckspin = null;
    public CRServo duckspinblue = null;
    public Servo distancearmservoR = null;
    public Servo distancearmservoL = null;

    //Give names to our IMU for our Programs
    public BNO055IMU imu = null;

    //Give names our Sensors for our Programs
    public DistanceSensor distanceR;
    public DistanceSensor distanceL;

    public ColorSensor blocksensor;
    public ColorSensor ballsensor;

    public boolean driver = false;

    public RevBlinkinLedDriver cargolights;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    //Create names for our Hardware Maps
    HardwareMap hwMap = null;
    LinearOpMode opMode = null;
    private ElapsedTime period = new ElapsedTime();
    //OdometryGlobalCoordinatePosition globalPositionUpdate;

    public Hardware_21_22() {
    }

    public void init(HardwareMap ahwMap, LinearOpMode aopMode) {
        hwMap = ahwMap;
        opMode = aopMode;



        //Name Motors for Config
        frontLeftMotor = hwMap.get(DcMotorEx.class, "front_left");
        frontRightMotor = hwMap.get(DcMotorEx.class, "front_right");
        rearLeftMotor = hwMap.get(DcMotorEx.class, "back_left");
        rearRightMotor = hwMap.get(DcMotorEx.class, "back_right");
        intakemotor = hwMap.get(DcMotor.class, "intakemotor");
        lifter = hwMap.get(DcMotor.class, "lifter");
        duckextend = hwMap.get(DcMotor.class, "duckextend");

        //Name Servos for Config
        elementarm = hwMap.get(Servo.class, "elementarm");
        lightsaber = hwMap.get(Servo.class, "lightsaber");
        elementclamp1 = hwMap.get(Servo.class, "elementclamp1");
        elementclamp2 = hwMap.get(Servo.class, "elementclamp2");
        duckspin = hwMap.get(CRServo.class, "duckspin");
        duckspinblue = hwMap.get(CRServo.class, "duckspinblue");
        distancearmservoR = hwMap.get(Servo.class, "distancearmservoR");
        distancearmservoL = hwMap.get(Servo.class, "distancearmservoL");

        cargolights = hwMap.get(RevBlinkinLedDriver.class, "cargolights");

        //Sensors
        distanceR = hwMap.get(DistanceSensor.class,"distanceR");
        distanceL = hwMap.get(DistanceSensor.class, "distanceL");
        //blocksensor = hwMap.get(ColorSensor.class, "blockyboy");
        //ballsensor = hwMap.get(ColorSensor.class, "ballyman");

        //Set Direction the Motors will turn
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);

        //Set Direction the Servos will turn
        elementarm.setDirection(Servo.Direction.FORWARD);
        lightsaber.setDirection(Servo.Direction.FORWARD);
        elementclamp2.setDirection(Servo.Direction.FORWARD);
        elementclamp1.setDirection(Servo.Direction.REVERSE);
        duckspin.setDirection(CRServo.Direction.FORWARD);
        duckspinblue.setDirection(CRServo.Direction.REVERSE);
        duckextend.setDirection(CRServo.Direction.REVERSE);
        distancearmservoL.setDirection(Servo.Direction.REVERSE);
        distancearmservoR.setDirection(Servo.Direction.FORWARD);

        //Set Init Power to Motors and apply Automatic Breaking
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakemotor.setPower(0);
        intakemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter.setPower(0);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckextend.setPower(0);
        duckspin.setPower(0);

        //Set Init Position to all servos
        distancearmservoL.setPosition(.02);
        distancearmservoR.setPosition(0);
        elementclamp1.setPosition(0);
        elementclamp2.setPosition(0);
        lifter.setTargetPosition(0);
        lightsaber.setPosition(0);
        elementarm.setPosition(0);

        //Set all motors that are using Servos to RUN_USING_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Touch sensors to not being used
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

    }

    // Setting height layers ONCE
    public int TipTop = 3100;
    public int layer3 = 1900;
    public int layer2 = 1100;
    public int bottom = 400;

    /*public void teleInit() {

        distancearmservoL.setPosition(.02);
        distancearmservoR.setPosition(0);
        elementclamp1.setPosition(0);
        elementclamp2.setPosition(0);
        lifter.setTargetPosition(0);
        lightsaber.setPosition(0);
        elementarm.setPosition(0);
    }

    public void autoInit() {

        distancearmservoL.setPosition(.7);
        distancearmservoR.setPosition(.68);
        elementclamp1.setPosition(.5);
        elementclamp2.setPosition(.5);
        lifter.setTargetPosition(0);
        lightsaber.setPosition(0);
        elementarm.setPosition(0);
    }*/

}