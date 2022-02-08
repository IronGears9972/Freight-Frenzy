package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp(name = "RedTeleOp", group = "Linear Opmode")
public class RedTeleOp extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();
	private ElapsedTime time = new ElapsedTime();
	private ElapsedTime ducktime = new ElapsedTime();

	double frontleft = 0;
	double rearleft = 0;
	double rearright = 0;
	double frontright = 0;

	boolean Dillon = false;

	boolean failSafe = false;

	public void runOpMode() {

		telemetry.addData("Say", "Hello Iron Gears");
		telemetry.update();

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

		while (!opModeIsActive()) {

			if (gamepad1.a) {
				Dillon = true;
			}
			else if (gamepad1.b) {
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

		robot.frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
		robot.frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
		robot.rearLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
		robot.rearRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

		while (opModeIsActive()) {

			if (failSafe == false) {
				robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				driver(Dillon);

				if (robot.lifter.getCurrentPosition() > -15 && robot.lifter.getCurrentPosition() < 30) {

					if (robot.distance3.getDistance(DistanceUnit.INCH) < 3) {
						time.reset();
						if (time.seconds() < 1) {
							robot.intakemotor.setPower(.95);
						}
					} else {
						if (time.seconds() > 1) {
							if (gamepad1.right_trigger == 1) {
								robot.intakemotor.setPower(-.95);
							} else if (gamepad1.left_trigger == 1) {
								robot.intakemotor.setPower(.95);
							} else if (gamepad2.right_trigger == 1) {
								robot.intakemotor.setPower(-.95);
							} else if (gamepad2.left_trigger == 1) {
								robot.intakemotor.setPower(.95);
							} else {
								robot.intakemotor.setPower(0);
							}
						}
					}
				}

				if (gamepad2.dpad_up && gamepad2.left_bumper) {
					robot.elementclamp1.setPosition(.5);
					robot.elementclamp2.setPosition(.5);
				}
				else {
					robot.elementclamp1.setPosition(0);
					robot.elementclamp2.setPosition(0);
				}

				if (gamepad2.y && !gamepad2.left_bumper) {
					ducktime.reset();
					while (gamepad2.y) {
						if (ducktime.seconds() < .5) {
							robot.duckspin.setPower(-.8);
						} else {
							robot.duckspin.setPower(-.95);
						}
					}
					robot.duckspin.setPower(.2);
				} else {
					robot.duckspin.setPower(0);
				}

				if (gamepad2.dpad_left) {
					robot.duckextend.setPower(.95);
				}
				else if (gamepad2.dpad_right) {
					robot.duckextend.setPower(-.95);
				}
				else {
					robot.duckextend.setPower(0);
				}

				if (!gamepad2.left_bumper && gamepad2.dpad_down && !gamepad2.right_bumper) {
					robot.elementarm.setPosition(.295);
				}
				else if (gamepad2.left_bumper && gamepad2.dpad_down && !gamepad2.right_bumper) {
					robot.elementarm.setPosition(0.355);
				}
				else if (gamepad2.right_bumper && gamepad2.dpad_down) {
					robot.elementarm.setPosition(0.25);
				}
				else if (!gamepad2.left_bumper && gamepad2.dpad_up) {
					robot.elementarm.setPosition(0);
				}

				if (gamepad2.x && !gamepad2.left_bumper) {
					robot.lightsaber.setPosition(0.6);
				} else {
					robot.lightsaber.setPosition(1);
				}


				if (!gamepad2.left_bumper && gamepad2.a) {
					robot.lifter.setTargetPosition(1175);
					robot.lifter.setPower(.9);
				} else if (!gamepad2.left_bumper && gamepad2.b) {
					robot.lifter.setTargetPosition(2815);
					robot.lifter.setPower(.9);
				} else if (gamepad2.left_bumper && gamepad2.a) {
					robot.lifter.setTargetPosition(3290);
					robot.lifter.setPower(.9);
				} else if (gamepad2.left_bumper && gamepad2.b) {
					robot.lifter.setTargetPosition(3500);
					robot.lifter.setPower(.9);
				} else if (gamepad2.left_bumper && gamepad2.right_bumper) {
					robot.lifter.setTargetPosition(0);
					robot.lifter.setPower(.9);
				}

				if (robot.lifter.getCurrentPosition() <= (robot.lifter.getTargetPosition() - 15) || robot.lifter.getCurrentPosition() >= (robot.lifter.getTargetPosition() + 15)) {
					robot.lifter.setPower(0.9);
				} else {
					robot.lifter.setPower(0);
				}


				if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1 &&
						gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
				{
					failSafe = true;
				}

			}

			//Fail safe if sensor breaks

			else {
				robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

				driver(Dillon);

				if (gamepad2.b) {
					robot.lifter.setPower(.9);
				}
				else if (gamepad2.a) {
					robot.lifter.setPower(-.9);
				}
				else {
					robot.lifter.setPower(0);
				}

				if (!gamepad2.left_bumper && gamepad2.dpad_down) {
					robot.elementarm.setPosition(.295);
				}
				else if (!gamepad2.left_bumper && gamepad2.dpad_down && !gamepad2.right_bumper) {
					robot.elementarm.setPosition(0.355);
				}
				else if (gamepad2.right_bumper && gamepad2.dpad_down) {
					robot.elementarm.setPosition(0.25);
				}
				else if (!gamepad2.left_bumper && gamepad2.dpad_up) {
					robot.elementarm.setPosition(0);
				}

				if (gamepad2.dpad_left) {
					robot.duckextend.setPower(.95);
				}
				else if (gamepad2.dpad_right) {
					robot.duckextend.setPower(-.95);
				}
				else {
					robot.duckextend.setPower(0);
				}

				if (gamepad2.x && !gamepad2.left_bumper) {
					robot.lightsaber.setPosition(0.55);
				}
				else {
					robot.lightsaber.setPosition(1);
				}

				if (gamepad2.y && !gamepad2.left_bumper) {
					ducktime.reset();
					while (gamepad2.y) {
						if (ducktime.seconds() < .5) {
							robot.duckspin.setPower(-.8);
						} else {
							robot.duckspin.setPower(-.95);
						}
					}
					robot.duckspin.setPower(.2);
				} else {
					robot.duckspin.setPower(0);
				}

				if (gamepad2.dpad_up && gamepad2.left_bumper) {
					robot.elementclamp1.setPosition(.5);
					robot.elementclamp2.setPosition(.5);
				}
				else {
					robot.elementclamp1.setPosition(0);
					robot.elementclamp2.setPosition(0);
				}

				if (gamepad1.right_trigger == 1) {
					robot.intakemotor.setPower(-.95);
				}
				else if (gamepad1.left_trigger == 1) {
					robot.intakemotor.setPower(.95);
				}
				else if (gamepad2.right_trigger == 1) {
					robot.intakemotor.setPower(-.95);
				}
				else if (gamepad2.left_trigger == 1) {
					robot.intakemotor.setPower(.95);
				}
				else {
					robot.intakemotor.setPower(0);
				}

			}

			if (runtime.seconds() > 0 && runtime.seconds() <= 80) {
				if (robot.blocksensor.argb() > -15280000 && robot.blocksensor.argb() < -15240000 ||
						robot.blocksensor.argb() > -15280000 && robot.blocksensor.argb() < -15240000 ||
						robot.blocksensor.argb() > -210000000 && robot.blocksensor.argb() < -200000000 ) {
					robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
				}
				else if (robot.blocksensor.argb() < 0) {
					robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
				}
				else {
					robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
				}
			}
			else if (runtime.seconds() > 80 && runtime.seconds() < 85) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
			}
			else if (runtime.seconds() >= 85 && runtime.seconds() < 90) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
			}
			else if (runtime.seconds() >= 90 && runtime.seconds() < 110) {
				if (robot.blocksensor.argb() > -15280000 && robot.blocksensor.argb() < -15240000 ||
						robot.blocksensor.argb() > -15280000 && robot.blocksensor.argb() < -15240000 ||
						robot.blocksensor.argb() > -210000000 && robot.blocksensor.argb() < -200000000 ) {
					robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
				}
				else if (robot.blocksensor.argb() < 0) {
					robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
				}
				else {
					robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
				}
			}
			else if (runtime.seconds() >= 110 && runtime.seconds() < 115) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
			}
			else if (runtime.seconds() >= 115 && runtime.seconds() < 120) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
			}
			else {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
			}

			telemetry.addData("Status", "Run Time: " + runtime.toString());

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

			telemetry.addData("BS-ARGB", robot.blocksensor.argb());

			telemetry.addData("boxSensor", robot.distance3.getDistance(DistanceUnit.INCH));
			telemetry.addData("colorDistance", robot.blocksensor_distance.getDistance(DistanceUnit.INCH));
			telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
			telemetry.addData("lifterTP", robot.lifter.getTargetPosition());
			telemetry.addData("Lift Power", robot.lifter.getPower());

			telemetry.addData("Intake Power", robot.intakemotor.getPower());
			telemetry.update();
		}
	}

	public void driver(boolean D) {


		double powermotor = .52;

		if (gamepad1.right_bumper) {
			powermotor = 1;
		} else if (gamepad1.left_bumper) {
			powermotor = .35;
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