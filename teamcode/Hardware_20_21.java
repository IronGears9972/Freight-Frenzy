package org.firstinspires.ftc.teamcode;

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


public class Hardware_20_21 {

    //Give names to our Motors for our Programs
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;
    public DcMotor intakemotor = null;
    public DcMotorEx launcher1 = null;
    public DcMotor conveyor = null;

    //Give names to our Servos for our Programs
    public Servo wobblehand = null;
    public Servo wobble = null;
    public Servo wobblehand2 = null;
    public Servo wobble2 = null;
    public Servo wobblekeep = null;
    public CRServo intakeservo = null;
    public Servo kicker = null;
    public Servo forks = null;
    public Servo lightsaber = null;

    //Give names to our Distance Sensors for our Programs
    public DistanceSensor DSLeftFront = null;
    public DistanceSensor DSLeftBack = null;
    public ModernRoboticsI2cRangeSensor DSRearLeft = null;
    public DistanceSensor DSRearRight = null;
    public DistanceSensor sensorDistance = null;

    //Give names to our Color Sensors for our Programs
    public ColorSensor sensorColor = null;
    public ColorSensor sensorColorL = null;
    public ColorSensor sensorColorR = null;

    //Give names to our IMU for our Programs
    public BNO055IMU imu = null;

    //Give names our Touch Sensors for our Programs
    public DigitalChannel digitalTouch;

    //Create a variable that we need for Odometry
    final double COUNTS_PER_INCH = 8192.0/(3.14*2);

    //Create names for our Hardware Maps
    HardwareMap hwMap = null;
    LinearOpMode opMode = null;
    private ElapsedTime period = new ElapsedTime();
    //OdometryGlobalCoordinatePosition globalPositionUpdate;

    public Hardware_20_21() {
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
        launcher1 = hwMap.get(DcMotorEx.class, "launcher1");
        conveyor = hwMap.get(DcMotor.class, "conveyor");

        //Name Servos for Config
        wobble = hwMap.get(Servo.class, "wobble");
        wobblekeep = hwMap.get(Servo.class, "wobblekeep");
        wobblehand = hwMap.get(Servo.class, "wobblehand");
        wobble2 = hwMap.get(Servo.class, "wobble2");
        wobblehand2 = hwMap.get(Servo.class, "wobblehand2");
        intakeservo = hwMap.get(CRServo.class, "intakeservo");
        kicker = hwMap.get(Servo.class, "kicker");
        forks = hwMap.get(Servo.class, "forks");
        lightsaber = hwMap.get(Servo.class, "lightsaber");

        //Name DistanceSensors for Config
        DSLeftFront = hwMap.get(DistanceSensor.class,"DSLeftFront");
        DSLeftBack = hwMap.get(DistanceSensor.class,"DSLeftBack");
        DSRearLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class,"DSRearLeft");
        DSRearRight = hwMap.get(DistanceSensor.class,"DSRearRight");

        //Name Color Sensors for Config
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColorL = hwMap.get(ColorSensor.class, "sensor_colorL");
        sensorColorR = hwMap.get(ColorSensor.class, "sensor_colorR");


        //Touch Sensors
        //digitalTouch = hwMap.get(DigitalChannel.class, "touchy");

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

        //Color Sensor Setup
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);


        Color.RGBToHSV((int) (sensorColorL.red() * SCALE_FACTOR),
                (int) (sensorColorL.green() * SCALE_FACTOR),
                (int) (sensorColorL.blue() * SCALE_FACTOR),
                hsvValues);


        Color.RGBToHSV((int) (sensorColorR.red() * SCALE_FACTOR),
                (int) (sensorColorR.green() * SCALE_FACTOR),
                (int) (sensorColorR.blue() * SCALE_FACTOR),
                hsvValues);



        //Set Direction the Motors will turn
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);
        launcher1.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        //Set Direction the Servos will turn
        wobble.setDirection(Servo.Direction.REVERSE);
        wobblehand.setDirection(Servo.Direction.REVERSE);
        wobblekeep.setDirection(Servo.Direction.REVERSE);
        wobble2.setDirection(Servo.Direction.FORWARD);
        wobblehand2.setDirection(Servo.Direction.FORWARD);
        intakeservo.setDirection(CRServo.Direction.FORWARD);
        kicker.setDirection(Servo.Direction.REVERSE);
        forks.setDirection(Servo.Direction.REVERSE);
        lightsaber.setDirection(Servo.Direction.REVERSE);

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

        launcher1.setPower(0);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeservo.setPower(0);
        conveyor.setPower(0);

        //Set Init Position to all servos
        kicker.setPosition(1);
        forks.setPosition(0.05);

        //Set all motors that are using Servos to RUN_USING_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Touch sensors to not being used
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        //Set Odometry to "IN USE"
        //globalPositionUpdate = new OdometryGlobalCoordinatePosition(frontRightMotor, rearLeftMotor, rearRightMotor, COUNTS_PER_INCH, 75);
        //Thread positionThread = new Thread(globalPositionUpdate);
        //positionThread.start();


    }

    //Testing Pure Pursuit

    /*public void gotoposition (double targetXPosition, double targetYposition, double robotPower, double desiredrobotOrientaion){





        double allowableDistanceError = 1.0;

        double distanceToXTarget = targetXPosition -globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYposition -globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opMode.opModeIsActive() && (distance > allowableDistanceError)){

            distanceToXTarget = targetXPosition -globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYposition -globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            double robotMovemoentAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovemoentAngle,robotPower);
            double robot_movement_y_component = calculateY(robotMovemoentAngle,robotPower);
            double pivotCorrection = desiredrobotOrientaion - globalPositionUpdate.returnOrientation();

            double turnpower = 0; // pivotCorrection / 90 * robotPower;

            double frontleft = (robot_movement_y_component - robot_movement_x_component - turnpower);
            double rearleft = (robot_movement_y_component + robot_movement_x_component - turnpower);
            double rearright = (robot_movement_y_component - robot_movement_x_component + turnpower);
            double frontright = (robot_movement_y_component + robot_movement_x_component + turnpower);



            frontLeftMotor.setPower(frontleft);
            rearLeftMotor.setPower(rearleft);
            rearRightMotor.setPower(rearright*1.03);
            frontRightMotor.setPower(frontright*1.03);


        }

        frontLeftMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        frontRightMotor.setPower(0);

    }

    public void gotopositionstrafe (double targetXPosition, double targetYposition, double robotPower, double desiredrobotOrientaion){



        double allowableDistanceError = 1.0;

        double distanceToXTarget = targetXPosition -globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYposition -globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opMode.opModeIsActive() && (distance > allowableDistanceError)){

            distanceToXTarget = targetXPosition -globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYposition -globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            double robotMovemoentAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovemoentAngle,robotPower);
            double robot_movement_y_component = calculateY(robotMovemoentAngle,robotPower);
            double pivotCorrection = desiredrobotOrientaion - globalPositionUpdate.returnOrientation();

            double turnpower = 0; // pivotCorrection / 90 * robotPower;

            double frontleft = (robot_movement_y_component - robot_movement_x_component - turnpower);
            double rearleft = (robot_movement_y_component + robot_movement_x_component - turnpower);
            double rearright = (robot_movement_y_component - robot_movement_x_component + turnpower);
            double frontright = (robot_movement_y_component + robot_movement_x_component + turnpower);



            frontLeftMotor.setPower(frontleft);
            rearLeftMotor.setPower(rearleft);
            rearRightMotor.setPower(rearright);
            frontRightMotor.setPower(frontright);


        }

        frontLeftMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        frontRightMotor.setPower(0);

    }

     */

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public double getangel(){
        Orientation angels = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angels.firstAngle;

    }

    public void drivestraight2(double inches, double power) {


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

            if (sensorColorL.blue() > sensorColorL.red()){

                break;

            }

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

    public void drivestraightleft(double inches, double power) {


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

                rearLeftMotor.setPower((power / 4)*2);
                rearRightMotor.setPower(power / 4);
                frontLeftMotor.setPower((power / 4)*2);
                frontRightMotor.setPower(power / 4);

            }else if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (13*(600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (13*(600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (13*(600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (13*(600 / 26)))) {

                rearLeftMotor.setPower((power / 2)*2);
                rearRightMotor.setPower(power / 2);
                frontLeftMotor.setPower((power / 2)*2);
                frontRightMotor.setPower(power / 2);

            }else if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (2.6*(600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (2.6*(600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (2.6*(600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (2.6*(600 / 26)))) {

                rearLeftMotor.setPower((power / 10)*2);
                rearRightMotor.setPower(power / 10);
                frontLeftMotor.setPower((power / 10)*2);
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

    public void drivestraightsonicsensor(double inches, double power,double distancefromwall) {


        double startangel = getangel();

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

            double distance = DSRearLeft.getDistance(DistanceUnit.INCH);

            opMode.telemetry.addData("Target  ", "%4d %4d %4d %4d", rearLeftMotor.getTargetPosition(), rearRightMotor.getTargetPosition(), frontLeftMotor.getTargetPosition(), frontRightMotor.getTargetPosition());
            opMode.telemetry.addData("Current ", "%4d %4d %4d %4d", rearLeftMotor.getCurrentPosition(), rearRightMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
            opMode.telemetry.addData("distance from wall","%.2f %.2f",distancefromwall,distance);
            opMode.telemetry.update();

            double strafepower = 0;

            if (distance<(distancefromwall-3)){

                strafepower = -1;

            }

            if (distance>(distancefromwall+3)){

                strafepower = 1;

            }

            double corentangel = getangel();
            double turnpower = 0;
            if (corentangel<(startangel-10)){
                turnpower = 1;

            }

            if (corentangel>(startangel+10)) {
                turnpower = -1;
            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < ((600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < ((600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < ((600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < ((600 / 26)))) {

                rearLeftMotor.setPower((power+strafepower + turnpower));
                rearRightMotor.setPower((power-strafepower - turnpower));
                frontLeftMotor.setPower((power-strafepower - turnpower));
                frontRightMotor.setPower((power+strafepower + turnpower));

            }

            /*

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (1.625 * (600 / 26)))) {

                rearLeftMotor.setPower(((power+strafepower + turnpower) / 16));
                rearRightMotor.setPower((power-strafepower - turnpower) / 16);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 16));
                frontRightMotor.setPower((power+strafepower + turnpower) / 16);

            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (1.625 * (600 / 26)))) {


                rearLeftMotor.setPower(((power+strafepower + turnpower) / 14));
                rearRightMotor.setPower((power-strafepower - turnpower) / 14);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 14));
                frontRightMotor.setPower((power+strafepower + turnpower) / 14);

            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (1.625 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (1.625 * (600 / 26)))) {


                rearLeftMotor.setPower(((power+strafepower + turnpower) / 12));
                rearRightMotor.setPower((power-strafepower - turnpower) / 12);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 12));
                frontRightMotor.setPower((power+strafepower + turnpower) / 12);

            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (2.6 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (2.6 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (2.6 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (2.6 * (600 / 26)))) {

                rearLeftMotor.setPower(((power+strafepower + turnpower) / 10));
                rearRightMotor.setPower((power-strafepower - turnpower) / 10);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 10));
                frontRightMotor.setPower((power+strafepower + turnpower) / 10);

            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (3.25 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (3.25 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (3.25 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (3.25 * (600 / 26)))) {


                rearLeftMotor.setPower(((power+strafepower + turnpower) / 8));
                rearRightMotor.setPower((power-strafepower - turnpower) / 8);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 8));
                frontRightMotor.setPower((power+strafepower + turnpower) / 8);

            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (4.333 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (4.333 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (4.333 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (4.333 * (600 / 26)))) {

                rearLeftMotor.setPower(((power+strafepower + turnpower) / 6));
                rearRightMotor.setPower((power-strafepower - turnpower) / 6);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 6));
                frontRightMotor.setPower((power+strafepower + turnpower) / 6);

            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (5.2 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (5.2 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (5.2 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (5.2 * (600 / 26)))) {

                rearLeftMotor.setPower(((power+strafepower + turnpower) / 5));
                rearRightMotor.setPower((power-strafepower - turnpower) / 5);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 5));
                frontRightMotor.setPower((power+strafepower + turnpower) / 5);

            }

            if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (8.66 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (8.66 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (8.66 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (8.66 * (600 / 26)))) {

                rearLeftMotor.setPower(((power+strafepower + turnpower) / 3));
                rearRightMotor.setPower((power-strafepower - turnpower) / 3);
                frontLeftMotor.setPower(((power-strafepower - turnpower) / 3));
                frontRightMotor.setPower((power+strafepower + turnpower) / 3);

            }


            else if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (6.5 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (6.5 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (6.5 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (6.5 * (600 / 26)))) {


                rearLeftMotor.setPower((power+strafepower + turnpower) / 4);
                rearRightMotor.setPower((power+strafepower + turnpower) / 4);
                frontLeftMotor.setPower((power+strafepower + turnpower) / 4);
                frontRightMotor.setPower((power+strafepower + turnpower) / 4);

            }
            else if ((Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (13 * (600 / 26)))&(Math.abs(frontRightMotor.getTargetPosition() - frontRightMotor.getCurrentPosition()) < (13 * (600 / 26)))&(Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < (13 * (600 / 26)))&(Math.abs(rearRightMotor.getTargetPosition() - rearRightMotor.getCurrentPosition()) < (13 * (600 / 26)))) {

                rearLeftMotor.setPower((power+strafepower + turnpower) / 2);
                rearRightMotor.setPower((power+strafepower + turnpower) / 2);
                frontLeftMotor.setPower((power+strafepower + turnpower) / 2);
                frontRightMotor.setPower((power+strafepower + turnpower) / 2);


            }

             */





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

    public void drivestraightsensor(double inches, double power,double distancefromwall) {

        double startangel = getangel();

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

            double distance = DSRearLeft.getDistance(DistanceUnit.INCH);

            opMode.telemetry.addData("Target  ", "%4d %4d %4d %4d", rearLeftMotor.getTargetPosition(), rearRightMotor.getTargetPosition(), frontLeftMotor.getTargetPosition(), frontRightMotor.getTargetPosition());
            opMode.telemetry.addData("Current ", "%4d %4d %4d %4d", rearLeftMotor.getCurrentPosition(), rearRightMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
            opMode.telemetry.addData("distance from wall","%.2f %.2f",distancefromwall,distance);
            opMode.telemetry.update();

            double strafepower = 0;

            if (distance<(distancefromwall-0.5)){

                strafepower = -0.025;

            }

            if (distance>(distancefromwall+0.5)){

                strafepower = 0.025;

            }

            double corentangel = getangel();
            double turnpower = 0;
            if (corentangel<(startangel-2)){
                turnpower = 0.1;

            }

            if (corentangel>(startangel+2)) {
                turnpower = -0.1;
            }
            if (Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (6.5 * (600 / 26))) {

                rearLeftMotor.setPower((power+strafepower + turnpower) / 4);
                rearRightMotor.setPower((power-strafepower - turnpower) / 4);
                frontLeftMotor.setPower((power-strafepower - turnpower) / 4);
                frontRightMotor.setPower((power+strafepower + turnpower) / 4);

            } else if (Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (13 * (600 / 26))) {

                rearLeftMotor.setPower((power+strafepower + turnpower) / 2);
                rearRightMotor.setPower((power-strafepower - turnpower) / 2);
                frontLeftMotor.setPower((power-strafepower - turnpower) / 2);
                frontRightMotor.setPower((power+strafepower + turnpower) / 2);

            } else if (Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (13 * (600 / 26))) {

                rearLeftMotor.setPower((power + strafepower + turnpower) / 0.01);
                rearRightMotor.setPower((power - strafepower - turnpower) / 0.01);
                frontLeftMotor.setPower((power - strafepower - turnpower) / 0.01);
                frontRightMotor.setPower((power + strafepower + turnpower) / 0.01);


            }else {

                rearLeftMotor.setPower(power + strafepower + turnpower);
                rearRightMotor.setPower(power - strafepower - turnpower);
            frontLeftMotor.setPower(power - strafepower - turnpower);
                frontRightMotor.setPower(power + strafepower + turnpower);

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

    public void drivestrafe2(double inches, double power) {


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

            if (sensorColor.blue() > sensorColor.red()){

                break;

            }

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

    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (opMode.opModeIsActive() && (delayTimer.seconds() < seconds)) {
            opMode.idle();
        }
    }

}