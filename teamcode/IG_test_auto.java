package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "IG_Test_Auto")
public class IG_test_auto extends LinearOpMode {

    Hardware_20_21 robot = new Hardware_20_21(); // use the class created to define a robot's hardware


    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;


    final double COUNTS_PER_INCH = 8192.0/(3.14*2);

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY

    String rfName = "front_right", rbName = "back_right", lfName = "front_left", lbName = "back_left";
    String horizontalEncoderName = rbName, verticalLeftEncoderName = rfName, verticalRightEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        gotoposition(0,95,0.4,0);

        robot.wobble.setPosition(0.25);
        sleep(500);
        robot.wobblehand.setPosition(0.5);
        sleep(500);

        gotoposition(-18,95,0.4,0);

        robot.wobble.setPosition(1);
        sleep(500);
        robot.wobblehand.setPosition(-0.3);
        sleep(500);

        gotoposition(-18,0,0.4,0);


        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate());
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate());
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void gotoposition (double targetXPosition, double targetYposition, double robotPower, double desiredrobotOrientaion){

        double allowableDistanceError = 1.0;

        double distanceToXTarget = targetXPosition -globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYposition -globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opModeIsActive() && (distance > allowableDistanceError)){

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


            left_front.setPower(frontleft);
            left_back.setPower(rearleft);
            right_back.setPower(rearright);
            right_front.setPower(frontright);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate());
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate());
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
            telemetry.addData("motor","left %.2f %.2f right %.2f %.2f",frontleft,rearleft,frontright,rearright);

            telemetry.addData("robotMovemoentAngle,","%.4f",robotMovemoentAngle);
            telemetry.addData("robot_movement_x_component,", "%.4f",robot_movement_x_component);
            telemetry.addData("robot_movement_y_component,", "%.4f",robot_movement_y_component);
            telemetry.addData("pivotCorrection,","%.4f",pivotCorrection);

            telemetry.addData("distance,","%.4f",distance);
            telemetry.addData("distanceToXTarget,","%.4f",distanceToXTarget);
            telemetry.addData("distanceToYTarget,","%.4f",distanceToYTarget);


            telemetry.update();

        }

        left_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
        right_front.setPower(0);

    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
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
}
