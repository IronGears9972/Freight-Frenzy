package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous(name="STRAFING", group="Pushbot")
public class bigstrafe extends LinearOpMode {

	/* Declare OpMode members. */
	Hardware_21_22 robot = new Hardware_21_22(); // use the class created to define a robot's hardware
	private ElapsedTime     runtime = new ElapsedTime();

	int layer = 0;
	static final double     FORWARD_SPEED = 0.6;
	static final double     TURN_SPEED    = 0.5;
	double DR = 0;
	double DL = 0;
	double targetL = 0;
	int dist = 0;
	@Override
	public void runOpMode() {
		/*
		 * Initialize the drive system variables.
		 * The init() method of the hardware class does all the work here
		 */

		// Send telemetry message to signify robot waiting;

		robot.init(hardwareMap, this);
		robot.duckextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		robot.duckextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.duckextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		telemetry.addData("CP", robot.duckextend.getCurrentPosition());
		telemetry.update();
		sleep(1);
		OpRunENCODERZero();
		int parking = 1;
		while (!opModeIsActive()){
			if (gamepad1.a){
				parking = 1;
			}
			else if (gamepad1.b){
				parking = 2;
			}
			else if (gamepad1.y){
				parking = 3;
			}
			telemetry.addData("Parking Pos.", parking);
			telemetry.update();
			if (isStopRequested()){
				break;
			}
		}

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		while (opModeIsActive()) {
		robot.drivestrafe(60,.25, "");
		robot.robotsleep(0);
		sleep(5000);
		robot.drivestrafe(-60,.25, "");
		robot.robotsleep(0);
		sleep(5000);
		robot.drivestrafe(60,.4, "");
		robot.robotsleep(0);
		sleep(5000);
		robot.drivestrafe(-60,.4, "");
		robot.robotsleep(0);
		sleep(5000);
		break;
		}
	}

	private void OpRunENCODERZero() {
		robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
	//              ^ Starts all Encoders
    /*
    private void OpRunTelemetryZero() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
        telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
        telemetry.addData("lifter", " %.0d",robot.lifter.getCurrentPosition());
        telemetry.update();
    }

     */
	//              ^ Outputs text so we can read data
}