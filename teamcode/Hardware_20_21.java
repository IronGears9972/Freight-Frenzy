package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;


public class Hardware_20_21 {


    /* Public OpMode members. */

    //PUT PHOTOS OF EVERY MOTOR SERVO AND VEX MOTOR

    //PUT HERE

    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;

    public DcMotor intakemotor = null;

    public DcMotorEx launcher1 = null;
    public DcMotorEx launcher2 = null;

    //public DcMotor wobble = null;
    //public DcMotor niu = null;

    public Servo wobblehand = null;
    public Servo wobble = null;

    public Servo wobblehand2 = null;
    public Servo wobble2 = null;

    public CRServo intakeservo = null;

    public DcMotor conveyor = null;

    //NEW
    public Servo kicker = null;
    public Servo forks = null;
    //NEW
    public DistanceSensor DSLeftFront = null;
    public DistanceSensor DSLeftBack = null;
    public ModernRoboticsI2cRangeSensor DSRearLeft = null;
    public DistanceSensor DSRearRight = null;

    public BNO055IMU imu = null;

    public DigitalChannel digitalTouch;

    final double COUNTS_PER_INCH = 8192.0/(3.14*2);

    /*

    public Servo niu5 = null;
    public Servo niu6 = null;

    public CRServo niu7 = null;
    public Servo niu8 = null;

    public CRServo niu9 = null;
    public CRServo niu10 = null;

    public CRServo niu11 = null;
    public CRServo niu12 = null;

    */


    //public ColorSensor colorleft = null;
    //public ColorSensor colorright = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    LinearOpMode opMode = null;
    private ElapsedTime period = new ElapsedTime();

    OdometryGlobalCoordinatePosition globalPositionUpdate;



    /* Constructor */
    public Hardware_20_21() {


    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode aopMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        opMode = aopMode;


        
        // Define and Initialize Motors
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right");
        rearLeftMotor = hwMap.get(DcMotor.class, "back_left");
        rearRightMotor = hwMap.get(DcMotor.class, "back_right");
        intakemotor = hwMap.get(DcMotor.class, "intakemotor");
        launcher1 = hwMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hwMap.get(DcMotorEx.class, "launcher2");
        wobble = hwMap.get(Servo.class, "wobble");
        //niu = hwMap.get(DcMotor.class, "niu");
        wobblehand = hwMap.get(Servo.class, "wobblehand");
        wobble2 = hwMap.get(Servo.class, "wobble2");
        wobblehand2 = hwMap.get(Servo.class, "wobblehand2");
        intakeservo = hwMap.get(CRServo.class, "intakeservo");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
        kicker = hwMap.get(Servo.class, "kicker");
        forks = hwMap.get(Servo.class, "forks");
        // DistanceSensor
        DSLeftFront = hwMap.get(DistanceSensor.class,"DSLeftFront");
        DSLeftBack = hwMap.get(DistanceSensor.class,"DSLeftBack");
        DSRearLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class,"DSRearLeft");
        DSRearRight = hwMap.get(DistanceSensor.class,"DSRearRight");

        digitalTouch = hwMap.get(DigitalChannel.class, "touchy");

        imu = hwMap.get(BNO055IMU.class,"imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // niu5 = hwMap.get(Servo.class, "niu5");
        // niu6 = hwMap.get(Servo.class, "niu6");
        //niu7 = hwMap.get(CRServo.class, "niu7");
        // niu8 = hwMap.get(Servo.class, "niu8");
        // niu9 = hwMap.get(CRServo.class, "niu9");
        //  niu10 = hwMap.get(CRServo.class, "niu10");
        // niu11 = hwMap.get(CRServo.class, "niu11");
        // niu12 = hwMap.get(CRServo.class, "niu12");
        //colorleft  = hwMap.get(ColorSensor.class, "colorleft");
        //colorright  = hwMap.get(ColorSensor.class, "colorright");




        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);
        launcher1.setDirection(DcMotor.Direction.FORWARD);
        launcher2.setDirection(DcMotor.Direction.FORWARD);
        //wobble.setDirection(DcMotor.Direction.REVERSE);
        //niu.setDirection(DcMotor.Direction.FORWARD);

        wobble.setDirection(Servo.Direction.FORWARD);
        wobblehand.setDirection(Servo.Direction.FORWARD);
        wobble2.setDirection(Servo.Direction.FORWARD);
        wobblehand2.setDirection(Servo.Direction.FORWARD);
        intakeservo.setDirection(CRServo.Direction.FORWARD);
        kicker.setDirection(Servo.Direction.REVERSE);
        forks.setDirection(Servo.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.FORWARD);


        //SET POWER TO ALL MOTORS AND CONTINUES SERVOS
        //SET POSITIONS TO ALL NORMAL SERVOS

        //ANDYMARK ORBITAL 3.7:1
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //TORQUENADO 60:1
        intakemotor.setPower(0);
        intakemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //GOBUILDA
        launcher1.setPower(0);
        launcher2.setPower(0);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //REV CORE HEX MOTORS
        //wobble.setPower(0);
        // niu.setPower(0);

        //VEX MOTORS (CONTINUES SERVOS)
        intakeservo.setPower(0);
        conveyor.setPower(0);
        // niu10.setPower(0);
        // niu11.setPower(0);


        //NORMAL SERVOS
        kicker.setPosition(1);
        forks.setPosition(0.05);
        // niu12.setPower(0);
        // niu4.setPosition(0.55);
        //niu5.setPosition(0);

        // niu6.setPosition(1);

        //NEW
        // niu3.setPosition(1);
        // niu4.setPosition(0.5);
        //NEW


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);



        //globalPositionUpdate = new OdometryGlobalCoordinatePosition(frontRightMotor, rearLeftMotor, rearRightMotor, COUNTS_PER_INCH, 75);
        //Thread positionThread = new Thread(globalPositionUpdate);
        //positionThread.start();


    }



    public void gotoposition (double targetXPosition, double targetYposition, double robotPower, double desiredrobotOrientaion){





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

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
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

            if (Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (6.5 * (600 / 26))) {

                rearLeftMotor.setPower(power / 4);
                rearRightMotor.setPower(power / 4);
                frontLeftMotor.setPower(power / 4);
                frontRightMotor.setPower(power / 4);

            } else if (Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (13 * (600 / 26))) {

                rearLeftMotor.setPower(power / 2);
                rearRightMotor.setPower(power / 2);
                frontLeftMotor.setPower(power / 2);
                frontRightMotor.setPower(power / 2);

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

            double distance = DSRearRight.getDistance(DistanceUnit.INCH);

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
            if (corentangel<(startangel-3)){
                turnpower = 0.1;

            }

            if (corentangel>(startangel+3)) {
                turnpower = -0.1;
            }

            if (Math.abs(rearLeftMotor.getTargetPosition() - rearLeftMotor.getCurrentPosition()) < (2.6 * (600 / 26))) {

                rearLeftMotor.setPower((power+strafepower + turnpower) / 10);
                rearRightMotor.setPower((power-strafepower - turnpower) / 10);
                frontLeftMotor.setPower((power-strafepower - turnpower) / 10);
                frontRightMotor.setPower((power+strafepower + turnpower) / 10);

            }

            /*

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


            }else {

                rearLeftMotor.setPower(power + strafepower + turnpower);
                rearRightMotor.setPower(power - strafepower - turnpower);
                frontLeftMotor.setPower(power - strafepower - turnpower);
                frontRightMotor.setPower(power + strafepower + turnpower);

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

            double distance = DSLeftFront.getDistance(DistanceUnit.INCH);

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
    public void drivestraightsensorLB(double inches, double power,double distancefromwall) {

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

            double distance = DSLeftBack.getDistance(DistanceUnit.INCH);

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
            if (corentangel<(startangel-5)){
                turnpower = 0.1;

            }

            if (corentangel>(startangel+5)) {
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

        rearLeftMotor.setPower(power*1.25);
        rearRightMotor.setPower(power*1.25);
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
    }

    public void driveturn(double inches, double power) {


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = (int) (inches * (600 / 26.0));
        // 500 = 90*
        rearLeftMotor.setTargetPosition(position);
        rearRightMotor.setTargetPosition(-position);
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


        while (rearLeftMotor.isBusy() && rearRightMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy()) {
            opMode.telemetry.addData("Target  ", "%4d %4d %4d %4d", rearLeftMotor.getTargetPosition(), rearRightMotor.getTargetPosition(), frontLeftMotor.getTargetPosition(), frontRightMotor.getTargetPosition());
            opMode.telemetry.addData("Current ", "%4d %4d %4d %4d", rearLeftMotor.getCurrentPosition(), rearRightMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);


    }

    public void trunangel (double angel, double power) {

        double startangel = getangel();

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (angel>0){

            rearLeftMotor.setPower(-power );
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


    }


    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (opMode.opModeIsActive() && (delayTimer.seconds() < seconds)) {
            opMode.idle();
        }
    }


}