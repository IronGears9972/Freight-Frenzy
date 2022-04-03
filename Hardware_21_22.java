package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public DcMotor duckspin = null;

    //Give names to our Servos for our Programs
    public Servo lightsaber = null;
    public CRServo tapeExtend = null;
    public Servo tapeRotate = null;


    //Give names to our IMU for our Programs
    public BNO055IMU imu = null;

    //Give names our Sensors for our Programs
    public DistanceSensor freightDetector;
    public DistanceSensor blocksensor_distance;
    public ColorSensor blocksensor;

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
        duckspin = hwMap.get(DcMotor.class, "duckspin");

        duckextend = hwMap.get(DcMotor.class, "duckextend");

        //Name Servos for Config
        lightsaber = hwMap.get(Servo.class, "lightsaber");
        tapeExtend = hwMap.get(CRServo.class, "tapeextension");
        tapeRotate = hwMap.get(Servo.class, "taperotate");

        cargolights = hwMap.get(RevBlinkinLedDriver.class, "cargolights");

        //Sensors
        blocksensor = hwMap.get(ColorSensor.class, "blockyboy");
        blocksensor_distance = hwMap.get(DistanceSensor.class, "blockyboy");
        freightDetector = hwMap.get(DistanceSensor.class, "blockyboy");

        //Set Direction the Motors will turn
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);

        //Set Direction the Servos will turn
        lightsaber.setDirection(Servo.Direction.REVERSE);
        duckextend.setDirection(CRServo.Direction.REVERSE);
        tapeRotate.setDirection(Servo.Direction.FORWARD);
        tapeExtend.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set Init Power to Motors and apply Automatic Breaking
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        rearLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakemotor.setPower(0);
        intakemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter.setPower(0);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckextend.setPower(0);
        duckspin.setPower(0);

        //Set Init Position to all servos
        lifter.setTargetPosition(0);
        lightsaber.setPosition(0);
        tapeRotate.setPosition(0);

        //Set all motors that are using Servos to RUN_USING_ENCODER
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //L
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //R
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //M
        intakemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set motors to run without Encoder
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    // Setting height layers ONCE
    public int TipTop = 3500;
    public int layer3 = 3290;
    public int layer2 = 2815;
    public int bottom = 1175;

    public int layer1A = 1050;
    public int layer2A = 1600;
    public int layer3A = 2725;

    public double down = 0.355;
    public double up = 0;
    public double closed = 0;
    public double open = 0.5;
    public double reading = 0.99;
    public double retreating = 0.3;

    int duckTarget = 1234;

    public void raiseToLayer(int layer){
        if(layer == 1){
            lifter.setTargetPosition(layer1A);
        }
        else if(layer == 2){
            lifter.setTargetPosition(layer2A);
        }
        else if(layer == 3){
            lifter.setTargetPosition(layer3A);
        }
        lifter.setPower(0.95);
    }

    public void backToZero(){
        lifter.setTargetPosition(0);
        lifter.setPower(0.95);
    }

    public void stopLift(){
        lifter.setPower(0);
    }

    public int read(boolean redSide, boolean leftRead, boolean rightRead){
        int layerOrPosition = 3;
        //vuforia moment
        //blah blah blah conditionals

        if(redSide){
            if(leftRead){
                layerOrPosition = 1;
            }
            else if(rightRead){
                layerOrPosition = 2;
            }
            else{
                layerOrPosition = 3;
            }
        }
        else{
            if(leftRead){
                layerOrPosition = 2;
            }
            else if(rightRead){
                layerOrPosition = 1;
            }
            else{
                layerOrPosition = 3;
            }
        }

        return layerOrPosition;
    }

    public void extend(){
        duckextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        duckextend.setTargetPosition(duckTarget);
        duckextend.setPower(0.95);
        while(duckextend.getCurrentPosition() < duckTarget){

        }


    }

}