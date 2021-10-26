package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Hardware_21_22 {

    //Give names to our Motors for our Programs
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;
    public DcMotor intakemotor = null;
    public DcMotor lifter = null;

    //Give names to our Servos for our Programs
    public CRServo barcodeR = null;
    public CRServo barcodeL = null;
    public Servo elementarm = null;
    public Servo lightsaber = null;
    public Servo elementclamp = null;
    public CRServo duckspin = null;
    public CRServo duckextend = null;

    //Give names to our IMU for our Programs
    public BNO055IMU imu = null;

    //Give names our Touch Sensors for our Programs
    public DigitalChannel BarcodeLeft;
    public DigitalChannel BarcodeRight;

    //Create a variable that we need for Odometry
    final double COUNTS_PER_INCH = 8192.0/(3.14*2);

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
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right");
        rearLeftMotor = hwMap.get(DcMotor.class, "back_left");
        rearRightMotor = hwMap.get(DcMotor.class, "back_right");
        intakemotor = hwMap.get(DcMotor.class, "intakemotor");
        lifter = hwMap.get(DcMotorEx.class, "lifter");

        //Name Servos for Config
        barcodeR = hwMap.get(CRServo.class, "barcodeR");
        barcodeL = hwMap.get(CRServo.class, "barcodeL");
        elementarm = hwMap.get(Servo.class, "elementarm");
        lightsaber = hwMap.get(Servo.class, "lightsaber");
        elementclamp = hwMap.get(Servo.class, "elementclamp");
        duckspin = hwMap.get(CRServo.class, "duckspin");
        duckextend = hwMap.get(CRServo.class, "duckextend");

        //Touch Sensors
        BarcodeRight = hwMap.get(DigitalChannel.class, "BarcodeRight");
        BarcodeLeft = hwMap.get(DigitalChannel.class, "BarcodeLeft");

        //IMU setup
        imu = hwMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Set Direction the Motors will turn
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);

        //Set Direction the Servos will turn
        barcodeL.setDirection(CRServo.Direction.REVERSE);
        barcodeR.setDirection(CRServo.Direction.REVERSE);
        elementarm.setDirection(Servo.Direction.REVERSE);
        lightsaber.setDirection(Servo.Direction.FORWARD);
        elementclamp.setDirection(Servo.Direction.FORWARD);
        duckspin.setDirection(CRServo.Direction.FORWARD);
        duckextend.setDirection(CRServo.Direction.REVERSE);

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

        barcodeL.setPower(0);
        barcodeR.setPower(0);
        duckextend.setPower(0);
        duckspin.setPower(0);

        //Set Init Position to all servos
        elementclamp.setPosition(0);
        elementarm.setPosition(0);
        lightsaber.setPosition(0);

        //Set all motors that are using Servos to RUN_USING_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Touch sensors to not being used
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

    }

    public double getangel(){
        Orientation angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angels.firstAngle;

    }

    public void drivestraight(double inches, double power) {


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = (int) (inches * (600 / 26.0));

        rearLeftMotor.setTargetPosition(position);
        rearRightMotor.setTargetPosition(position);
        frontLeftMotor.setTargetPosition(position);
        frontRightMotor.setTargetPosition(position);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);

        while (rearLeftMotor.isBusy() && rearRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Target  ", "%4d %4d %4d %4d", rearLeftMotor.getTargetPosition(), rearRightMotor.getTargetPosition(), frontLeftMotor.getTargetPosition(), frontRightMotor.getTargetPosition());
            opMode.telemetry.addData("Current ", "%4d %4d %4d %4d", rearLeftMotor.getCurrentPosition(), rearRightMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
            opMode.telemetry.update();

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (6.5*(600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (6.5*(600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (6.5*(600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (6.5*(600 / 26)))) {

                rearLeftMotor.setPower(power / 4);
                rearRightMotor.setPower(power / 4);
                frontLeftMotor.setPower(power / 4);
                frontRightMotor.setPower(power / 4);

            }else if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (13*(600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (13*(600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (13*(600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (13*(600 / 26)))) {

                rearLeftMotor.setPower(power / 2);
                rearRightMotor.setPower(power / 2);
                frontLeftMotor.setPower(power / 2);
                frontRightMotor.setPower(power / 2);

            }else if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (2.6*(600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (2.6*(600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (2.6*(600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (2.6*(600 / 26)))) {

                rearLeftMotor.setPower(power / 10);
                rearRightMotor.setPower(power / 10);
                frontLeftMotor.setPower(power / 10);
                frontRightMotor.setPower(power / 10);

            }

            opMode.idle();

        }

        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drivestrafe(double inches, double power) {


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = (int) (inches * (600 / 26.0));


        rearLeftMotor.setTargetPosition(-position);
        rearRightMotor.setTargetPosition(position);
        frontLeftMotor.setTargetPosition(position);
        frontRightMotor.setTargetPosition(-position);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);


        while (rearLeftMotor.isBusy() && rearRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Target  ", "%4d %4d %4d %4d", rearLeftMotor.getTargetPosition(), rearRightMotor.getTargetPosition(), frontLeftMotor.getTargetPosition(), frontRightMotor.getTargetPosition());
            opMode.telemetry.addData("Current ", "%4d %4d %4d %4d", rearLeftMotor.getCurrentPosition(), rearRightMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void trunangel (double angel, double power) {

        double startangel = getangel();

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (angel>0){

            rearLeftMotor.setPower(-power);
            rearRightMotor.setPower(power);
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);

        }
        else {
            rearLeftMotor.setPower(power );
            rearRightMotor.setPower(-power);
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
        }

        double targetangel = startangel + angel;
        double curentangel = getangel();
        double diff = targetangel - curentangel;

        while (Math.abs(diff)>2){

            targetangel = startangel + angel;
            curentangel = getangel();
            diff = targetangel - curentangel;

            opMode.telemetry.addData("target","%.2f",targetangel);
            opMode.telemetry.addData("curent","%.2f",curentangel);
            opMode.telemetry.addData("diff","%.2f",diff);


        }

        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void robotsleep(double power) {

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(power);

        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

}