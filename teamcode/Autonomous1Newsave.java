
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Disabled

@Autonomous(name="Autonomous1Newsave", group="Pushbot")
public class Autonomous1Newsave extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_21_22 robot = new Hardware_21_22(); // use the class created to define a robot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;


        robot.init(hardwareMap, this);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!opModeIsActive()) {


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
            telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
            telemetry.addData("launcher1", " %.0f", robot.launcher1.getVelocity() / 28 * 60);
            telemetry.addData("DSRearLeft", String.format("%.01f in", robot.DSRearLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftBack", String.format("%.01f in", robot.DSLeftBack.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftFront", String.format("%.01f in", robot.DSLeftFront.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSRearRight", String.format("%.01f in", robot.DSRearRight.getDistance(DistanceUnit.INCH)));
            Orientation angels = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("imu , first/second/third", "%.1f %.1f %.1f", angels.firstAngle, angels.secondAngle, angels.thirdAngle);
            telemetry.update();


        }

        //STEP 1
        // Drive to the ring and shot

        robot.wobblehand2.setPosition(0.35);
        robot.drivestraight(-25, 0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(7,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestraight(-35, 0.3);

        robot.drivestrafe(-3,0.3);

        robot.robotsleep(0);
        sleep(200);

        //robot.trunangel(1,0.5);

        robot.launcher1.setPower(0.80);
        sleep(1100);

        robot.launcher1.setPower(0.80);

        robot.conveyor.setPower(1);

        robot.forks.setPosition(0.24);
        sleep(250);

        robot.kicker.setPosition(0.6);
        sleep(250);

        robot.kicker.setPosition(1);
        sleep(50);

        robot.conveyor.setPower(1);
        sleep(750);

        robot.forks.setPosition(0.30);
        sleep(350);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(1);
        sleep(1300);

        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);

        robot.forks.setPosition(0.05);
        sleep(350);

        robot.kicker.setPosition(1);
        sleep(100);

        //------------------------------------------------------------------------------------------
        //STEP 2
        // Drive turn and drop the wobble

        //robot.trunangel(-1,0.5);

        /*

        robot.drivestrafe(-8,0.3);

        robot.intakemotor.setPower(-1);
        robot.intakeservo.setPower(0.8);

        robot.drivestraight(-12, 0.3);

        robot.robotsleep(0);
        sleep(2000);

        //robot.trunangel(-2,0.5);

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);

        robot.launcher1.setPower(0.75);
        sleep(1000);

        robot.launcher1.setPower(0.75);

        robot.conveyor.setPower(1);

        robot.forks.setPosition(0.30);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(300);

        robot.robotsleep(0);
        sleep(1000);

        //robot.trunangel(2,0.5);


         */

        robot.drivestraight(-55, 0.3);

        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);

        robot.drivestrafe(-20,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.wobble2.setPosition(0.15);
        sleep(800);

        robot.wobblehand2.setPosition(1);
        sleep(500);

        robot.robotsleep(0);
        sleep(1000);

        //=============================================

        robot.drivestrafe(-20,0.3);

        robot.robotsleep(0);
        sleep(200);


        robot.drivestraight(114,0.3);

        robot.wobblehand2.setPosition(0.35);
        sleep(500);

        robot.wobble2.setPosition(0.40);

        robot.drivestrafe(21,0.3);

        robot.drivestraight(-25,0.3);


        robot.drivestrafe(-8,0.3);

        robot.intakemotor.setPower(-1);
        robot.intakeservo.setPower(0.8);

        robot.drivestraight(-12, 0.3);

        robot.robotsleep(0);
        sleep(2000);

        //robot.trunangel(-2,0.5);

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);

        robot.launcher1.setPower(0.75);
        sleep(1000);

        robot.launcher1.setPower(0.75);

        robot.conveyor.setPower(1);

        robot.forks.setPosition(0.30);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(300);

        robot.robotsleep(0);
        sleep(1000);


    }
}
