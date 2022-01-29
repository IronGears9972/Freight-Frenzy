package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "GearsTeleOp", group = "Linear Opmode")
public class TestingTeleOp extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	double frontleft = 0;
	double rearleft = 0;
	double rearright = 0;
	double frontright = 0;

	boolean Dillon = false;
	boolean newX;
	boolean oldX;
	boolean armDown;

	ElapsedTime time;

	public void runOpMode() {


		telemetry.addData("Say", "Hello Iron Gears");
		telemetry.update();

		while (!opModeIsActive()) {

			if (gamepad1.a) {
				Dillon = true;
			} else if (gamepad1.b) {
				Dillon = false;
			}

			telemetry.addData("Dillon", Dillon);
			telemetry.update();

			if(isStopRequested()){
				break;
			}

		}

		waitForStart();
		runtime.reset();
		robot.init(hardwareMap, this);

		float hsvValues[] = {0F, 0F, 0F};

		final float values[] = hsvValues;

		robot.lifter.setTargetPosition(0);
		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		robot.duckextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		robot.lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		while (opModeIsActive()) {

			driver(Dillon);

			if (gamepad2.right_trigger == 1) {
				robot.intakemotor.setPower(-.95);
			} else if (gamepad2.left_trigger == 1) {
				robot.intakemotor.setPower(.95);
			} else {
				robot.intakemotor.setPower(0);
			}

			if (gamepad1.right_trigger == 1) {
				robot.intakemotor.setPower(-.95);
			} else if (gamepad1.left_trigger == 1) {
				robot.intakemotor.setPower(.95);
			} else {
				robot.intakemotor.setPower(0);
			}

			if (gamepad2.dpad_up) {
				robot.elementclamp1.setPosition(.5);
				robot.elementclamp2.setPosition(.5);
			} else {
				robot.elementclamp1.setPosition(0);
				robot.elementclamp2.setPosition(0);
			}

			if (gamepad2.y && !gamepad2.left_bumper) {
				while (gamepad2.y) {
					robot.duckspin.setPower(0.9);
					robot.duckspinblue.setPower(0.9);
				}
				robot.duckspin.setPower(-.2);
				robot.duckspinblue.setPower(-.2);
			} else {
				robot.duckspin.setPower(0);
				robot.duckspinblue.setPower(0);
			}

			if (gamepad2.dpad_left) {
				robot.duckextend.setPower(.9);
			} else if (gamepad2.dpad_right) {
				robot.duckextend.setPower(-.9);
			} else {
				robot.duckextend.setPower(0);
			}

			newX = gamepad2.dpad_down;
			if (newX && !oldX) {
				if (!armDown) {
					robot.elementarm.setPosition(.3);
					armDown = true;
				}
				else {
					robot.elementarm.setPosition(0);
					armDown = false;
				}
				oldX = newX;
			}

			//dpad down to set element arm down and open clam
			//dpad up to bring element arm up and close clam

			/*if (gamepad2.dpad_down && time.seconds() > .25) {
				armDown = !armDown;
				time.reset();
			}
			if (armDown) {
				robot.elementarm.setPosition(.3);
				robot.elementclamp.setPosition(0);
			}
			else {
				robot.elementarm.setPosition(0);
				robot.elementclamp.setPosition(.5);
			}*/

			if (gamepad2.x && !gamepad2.left_bumper) {
				robot.lightsaber.setPosition(0.6);
			} else {
				robot.lightsaber.setPosition(1);
			}

			if(!gamepad2.left_bumper && gamepad2.a){
				robot.lifter.setTargetPosition(300);
				robot.lifter.setPower(.9);
			}
			else if(!gamepad2.left_bumper && gamepad2.b){
				robot.lifter.setTargetPosition(640);
				robot.lifter.setPower(.9);
			}
			else if(!gamepad2.left_bumper && gamepad2.left_bumper && gamepad2.a){
				robot.lifter.setTargetPosition(1075);
				robot.lifter.setPower(.9);
			}
			else if(!gamepad2.left_bumper && gamepad2.left_bumper && gamepad2.b){
				robot.lifter.setTargetPosition(1125);
				robot.lifter.setPower(.9);
			}
			else if(gamepad2.left_bumper && gamepad2.right_bumper){
				robot.lifter.setTargetPosition(0);
				robot.lifter.setPower(.9);
			}
			else {
				robot.lifter.setPower(0);
			}

			telemetry.addData("leftWheel", robot.frontRightMotor.getCurrentPosition());
			telemetry.addData("rightWheel", robot.rearLeftMotor.getCurrentPosition());
			telemetry.addData("middleWheel", robot.frontLeftMotor.getCurrentPosition());

			telemetry.addData("FL", robot.frontLeftMotor.getPower());
			telemetry.addData("FR", robot.frontRightMotor.getPower());
			telemetry.addData("RL", robot.rearLeftMotor.getPower());
			telemetry.addData("RR", robot.rearRightMotor.getPower());

			telemetry.addData("FL", frontleft);
			telemetry.addData("FR", frontright);
			telemetry.addData("RL", rearleft);
			telemetry.addData("RR", rearright);

			telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
			telemetry.addData("lifterTP", robot.lifter.getTargetPosition());
			telemetry.addData("Lift Power", robot.lifter.getPower());

			telemetry.addData("Intake Power", robot.intakemotor.getPower());
			telemetry.update();
		}
	}

	public void driver(boolean D) {


		double powermotor = .4;

		if (gamepad1.right_bumper) {
			powermotor = 1;
		} else if (gamepad1.left_bumper) {
			powermotor = .15;
		}

		if (D) {
			rearleft = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
			frontleft = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
			frontright = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
			rearright = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
		}
		else {
			frontleft = (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
			rearleft = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
			rearright = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;
			frontright = (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;
		}

		robot.frontLeftMotor.setPower(frontleft);
		robot.rearLeftMotor.setPower(rearleft);
		robot.rearRightMotor.setPower(rearright);
		robot.frontRightMotor.setPower(frontright);

	}
}