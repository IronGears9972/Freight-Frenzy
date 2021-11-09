/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode;

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

@Autonomous(name="Blue Side", group="Pushbot")
public class AutoBlue extends LinearOpMode {

	/* Declare OpMode members. */
	Hardware_21_22 robot = new Hardware_21_22(); // use the class created to define a robot's hardware
	private ElapsedTime     runtime = new ElapsedTime();


	int layer;
	static final double     FORWARD_SPEED = 0.6;
	static final double     TURN_SPEED    = 0.5;
	double DR;
	double DL;
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


		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		//-------------------------------------------------------------------------------------

		robot.duckextend.setTargetPosition(1080);
		robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		while (robot.duckextend.getCurrentPosition() < 1080 ) {
			telemetry.addData("CP", robot.duckextend.getCurrentPosition());
			telemetry.update();
			robot.duckextend.setPower(-0.9);
			sleep(10);
		}
		sleep(100);
		robot.duckextend.setPower(0);

		for (int x = 0; x < 10; ++ x){
			DR = DR + robot.distanceR.getDistance(DistanceUnit.INCH);
			DL = DL + robot.distanceL.getDistance(DistanceUnit.INCH);
			telemetry.addData("DR", DR);
			telemetry.addData("DL", DL);
			telemetry.update();
			sleep(100);
		}
		DR = DR / 10;
		DL = DL / 10;

		if (DL > 17 && DL < 21) {
			layer = 1;
		} else if (DR > 17 && DR < 21) {
			layer = 2;
		} else {
			layer = 3;
		}

		telemetry.addData("laYER", layer);
		telemetry.update();

		robot.duckspin.setPower(0.8);
		sleep(2500);
		robot.duckspin.setPower(0);

		//-------------------------------------------------------------------------------------

		robot.duckextend.setTargetPosition(100);
		robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.duckextend.setPower(0.8);
		sleep(1000);
		robot.drivestraight(8,0.25);
		robot.robotsleep(0);
		sleep(500);
		robot.duckextend.setPower(0);

		int dist = 0;

		if (layer == 1){
			robot.drivestrafe(-4, 0.15);
			dist = -4;
		}
		else if (layer == 2){
			robot.drivestrafe(5, 0.15);
			dist = 5;
		}
		else if (layer == 3){
			robot.drivestrafe(12,0.15);
			dist = 12;
		}
		//-------------------------------------------------------------------------------------

		robot.elementarm.setPosition(0.05);
		robot.elementclamp.setPosition(0.5);
		sleep(50);
		robot.drivestraight(9.25, 0.15);
		sleep(250);
		robot.elementclamp.setPosition(0);
		sleep(750);
		robot.elementarm.setPosition(0.45);
		sleep(250);

		//-------------------------------------------------------------------------------------

		robot.drivestrafe(20-(dist), 0.15);
		robot.robotsleep(0);
		sleep(500);

		//-------------------------------------------------------------------------------------

		int targetL = 0;
		robot.drivestraight(8, 0.15);
		robot.robotsleep(0);
		if (layer == 1) {
			targetL = 100;
		}
		else if (layer == 2) {
			targetL = 420;
		}
		else if (layer == 3) {
			targetL = 720;
		}

		while (robot.lifter.getCurrentPosition() < targetL){
			robot.lifter.setPower(0.95);
			telemetry.addData("Current-Lift", robot.lifter.getCurrentPosition());
			telemetry.addData("Target-Lift", targetL);
			telemetry.update();
			sleep(1);
		}
		robot.lifter.setPower(0);
		sleep(250);
		robot.lightsaber.setPosition(0.5);

		//-------------------------------------------------------------------------------------

		robot.drivestraight(-24, 0.15);
		robot.robotsleep(0);
		sleep(1000);
		robot.drivestrafe(40, 0.15);

		//-------------------------------------------------------------------------------------



		//-------------------------------------------------------------------------------------

		OpRunTelemetryZero();
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

	private void OpRunTelemetryZero() {
		telemetry.addData("Status", "Run Time: " + runtime.toString());
		telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
		telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
		telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
		telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
		telemetry.addData("lifter", " %.0f",robot.lifter.getCurrentPosition());
		telemetry.update();
	}
	//              ^ Outputs text so we can read data




}


