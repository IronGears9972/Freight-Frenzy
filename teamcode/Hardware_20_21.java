package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

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

    public DcMotorEx launcher = null;

    //public DcMotor wobble = null;
    //public DcMotor niu = null;

    public Servo wobblehand = null;
    public Servo wobble = null;

    public CRServo intakeservo = null;

    public DcMotor conveyor = null;

    //NEW
    public Servo kicker = null;
    public Servo forks = null;
    //NEW


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
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        wobble = hwMap.get(Servo.class, "wobble");
        //niu = hwMap.get(DcMotor.class, "niu");
        wobblehand = hwMap.get(Servo.class, "wobblehand");
        intakeservo = hwMap.get(CRServo.class, "intakeservo");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
        kicker = hwMap.get(Servo.class, "kicker");
        forks = hwMap.get(Servo.class, "forks");

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
        launcher.setDirection(DcMotor.Direction.FORWARD);
        //wobble.setDirection(DcMotor.Direction.REVERSE);
        //niu.setDirection(DcMotor.Direction.FORWARD);

        wobble.setDirection(Servo.Direction.FORWARD);
        wobblehand.setDirection(Servo.Direction.FORWARD);
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


        //TORQUENADO 60:1
        intakemotor.setPower(0);
        intakemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //GOBUILDA
        launcher.setPower(0);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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



        globalPositionUpdate = new OdometryGlobalCoordinatePosition(frontRightMotor, rearLeftMotor, rearRightMotor, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


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


    /*


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




     */

}