package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Q2 Blandside", group="Pushbot")
public class LameAuto extends LinearOpMode {

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
			if (gamepad1.a || gamepad2.a){
				parking = 1;
			}
			else if (gamepad1.b || gamepad2.b){
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

			for (int x = 0; x < 10; ++x) {
				DR = DR + robot.distanceR.getDistance(DistanceUnit.INCH);
				DL = DL + robot.distanceL.getDistance(DistanceUnit.INCH);
				telemetry.addData("DR", DR);
				telemetry.addData("DL", DL);
				telemetry.update();
				sleep(10);
			}

			// Averages the two distance readings to account for any misreadings, then
			DR = DR / 10;
			DL = DL / 10;

			if (DL > 17 && DL < 24) {
				layer = 1;
			} else if (DR > 17 && DR < 24) {
				layer = 2;
			} else {
				layer = 3;
			}

			telemetry.addData("laYER", layer);
			telemetry.update();

			robot.drivestraight(9, 0.2);
			robot.robotsleep(0);
			sleep(700);

			if (layer == 1) {

				robot.drivestrafe(-4, 0.2, "N/A");
				dist = -4;

			}
			else if (layer == 2) {

				robot.drivestrafe(4, 0.2, "N/A");
				dist = 4;

			}

			robot.robotsleep(0);
			sleep(1000);
			robot.elementarm.setPosition(0.04);
			robot.elementclamp.setPosition(0.5);
			sleep(500);

			robot.drivestraight(10, 0.15);
			robot.robotsleep(0);
			sleep(500);
			robot.elementclamp.setPosition(0);
			sleep(900);
			robot.elementarm.setPosition(0.35);
			sleep(500);

			robot.drivestraight(-21, 0.25);
			robot.robotsleep(0);
			sleep(500);

			if (parking == 1) {
				robot.drivestrafe(25-(dist), 0.2, "S1");
				robot.robotsleep(0);
				sleep(500);
			}
			else if(parking == 2){
				robot.drivestrafe(-(25+dist), 0.2, "S1");
				robot.robotsleep(0);
				sleep(500);
			}

			robot.drivestraight(16, 0.15);
			robot.robotsleep(0);
			sleep(500);

			if (parking == 1) {
				robot.drivestrafe(15, 0.15, "S1");
				robot.robotsleep(0);
				sleep(500);
			}
			else if(parking == 2){
				robot.drivestrafe(-15, 0.15, "S1");
				robot.robotsleep(0);
				sleep(500);
			}

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