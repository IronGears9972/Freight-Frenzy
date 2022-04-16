package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.code.Carousel;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "RedTeleOp", group = "A")
public class RedTeleOp extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();
	private ElapsedTime time = new ElapsedTime();
	private ElapsedTime ducktime = new ElapsedTime();
	private ElapsedTime lightsabertime = new ElapsedTime();
	private ElapsedTime bigTime = new ElapsedTime();

	double frontleft = 0;
	double rearleft = 0;
	double rearright = 0;
	double frontright = 0;

	boolean closed = false;

	boolean sauce = false;
	boolean big = false;

	boolean Dillon = false;

	boolean failSafe = false;

	boolean down;

	int duckTarget;

	double x = .62;
	double x1 = 0;

	Carousel cl;

	public void runOpMode() {

		telemetry.addData("Say", "Hello Iron Gears");
		telemetry.update();

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

		cl = new Carousel(this);
		double speed = -40 ;//23

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

			drive.update();
			telemetry.addData("Position", "\nX:" + drive.getPoseEstimate().getX() + "\nY:" + drive.getPoseEstimate().getY() + "\nHeading:" + drive.getPoseEstimate().getHeading());

			if (failSafe == false) {
				robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				driver(Dillon);

				if (robot.lifter.getCurrentPosition() > -15 && robot.lifter.getCurrentPosition() < 30) {
					if (robot.blocksensor_distance.getDistance(DistanceUnit.INCH) < 2) {

						time.reset();

						if (time.seconds() < 1) {
							robot.intakemotor.setPower(.8);
						} else {
							robot.intakemotor.setPower(0);
						}
					}
					else {
						if (time.seconds() > 1) {
							if (gamepad1.right_trigger >= .6) {
								robot.intakemotor.setPower(-.95);
							} else if (gamepad1.left_trigger >= .6) {
								robot.intakemotor.setPower(.95);
							}  else {
								robot.intakemotor.setPower(0);
							}
						}
					}
				}
				else {
					robot.intakemotor.setPower(0);
				}



				if (gamepad2.y) {
					cl.startWheel(speed);
				}
				else {
					cl.stopWheel(6.5);
				}



				if (gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad1.dpad_right || gamepad1.dpad_left ||
						gamepad1.dpad_up || gamepad1.dpad_down) {
					duckTarget = 40;
				}
				else if (gamepad2.dpad_left || gamepad2.dpad_right) {
					duckTarget = -50;
				}



				if (gamepad2.dpad_left && !gamepad2.left_bumper) {
					robot.duckextend.setPower(.95);
					duckTarget = -100;
				}
				else if (gamepad2.dpad_right && !gamepad2.left_bumper) {
					robot.duckextend.setPower(-.95);
					duckTarget = -100;
				}
				else if (robot.duckextend.getCurrentPosition() < duckTarget) {
					robot.duckextend.setPower(.75);
				}
				else {
					robot.duckextend.setPower(0);
				}



				if (gamepad2.x) {
					robot.lightsaber.setPosition(0.48);
					if (closed) {
						closed = false;
						lightsabertime.reset();
					}
				} else {
					if (!closed && lightsabertime.seconds() > 1) {
						robot.lightsaber.setPosition(.06);
					}
				}



				if (!gamepad2.left_bumper && gamepad2.a) {
					robot.lifter.setTargetPosition(1175);
					robot.lifter.setPower(.95);
				}
				else if (!gamepad2.left_bumper && gamepad2.b) {
					robot.lifter.setTargetPosition(2815);
					robot.lifter.setPower(.95);
				}
				else if (gamepad2.left_bumper && gamepad2.a) {
					robot.lifter.setTargetPosition(3290);
					robot.lifter.setPower(.95);
				}
				else if (gamepad2.left_bumper && gamepad2.right_bumper) {
					robot.lifter.setTargetPosition(0);
					robot.lifter.setPower(.95);
					down = true;
				}

				if (down) {
					if (robot.lifter.getCurrentPosition() > 175 || robot.lifter.getCurrentPosition() < 800) {
						robot.lightsaber.setPosition(.45);
					}

					if (robot.lifter.getCurrentPosition() < 300) {
						robot.lightsaber.setPosition(0);
						down = false;
					}
				}

				if((gamepad1.x && gamepad1.left_bumper)){
					big = false;
				}

				if (gamepad1.x && !gamepad1.left_bumper) {
					robot.capper.setPosition(.4);
					big = true;
				}
				else if (!big) {
					robot.capper.setPosition(.02);
				}
				else {
					robot.capper.setPosition(.22);
				}



				if (robot.lifter.getCurrentPosition() <= (robot.lifter.getTargetPosition() - 15) || robot.lifter.getCurrentPosition() >= (robot.lifter.getTargetPosition() + 15)) {
					robot.lifter.setPower(0.9);
				} else {
					robot.lifter.setPower(0);
				}



				if (robot.duckextend.getCurrentPosition() >= 30) {
					if (gamepad1.dpad_left && !gamepad1.right_bumper && !gamepad1.left_bumper) {
						if (x <= .62) {
							x += .01;
						}
					} else if (gamepad1.dpad_right && !gamepad1.right_bumper && !gamepad1.left_bumper) {
						if (x >= 0) {
							x -= .01;
						}
					} else if (gamepad1.dpad_left && gamepad1.left_bumper) {
						if (x <= .62) {
							x += .002;
						}
					} else if (gamepad1.dpad_right && gamepad1.left_bumper) {
						if (x >= 0) {
							x -= .002;
						}
					} else if (gamepad1.dpad_left && gamepad1.right_bumper) {
						if (x <= .62) {
							x += .025;
						}
					} else if (gamepad1.dpad_right && gamepad1.right_bumper) {
						if (x >= 0) {
							x -= .025;
						}
					}



					/*if (gamepad1.dpad_up && !gamepad1.right_bumper && !gamepad1.left_bumper) {
						if (y <= 0.5) {
							y += .01;
						}
					}
					else if (gamepad1.dpad_down && !gamepad1.right_bumper && !gamepad1.left_bumper) {
						if (y >= .18) {
							y -= .01;
						}
					}
					else if (gamepad1.dpad_up && !gamepad1.right_bumper && gamepad1.left_bumper) {
						if (y <= 0.5) {
							y += .005;
						}
					}
					else if (gamepad1.dpad_down && !gamepad1.right_bumper && gamepad1.left_bumper) {
						if (y >= .18) {
							y -= .005;
						}
					}*/

					if (gamepad1.dpad_up) {
						robot.tapeUpDown2.setPower(.85);
					}
					else if (gamepad1.dpad_down) {
						robot.tapeUpDown2.setPower(-.85);
					}
					else if (gamepad1.dpad_up && gamepad1.left_bumper) {
						robot.tapeUpDown2.setPower(.5);
					}
					else if (gamepad1.dpad_down && gamepad1.left_bumper) {
						robot.tapeUpDown2.setPower(-.5);
					}
					else {
						robot.tapeUpDown2.setPower(.02);
					}



					if (gamepad1.right_stick_button) {
						x = .42;
					}
					else if (gamepad1.left_stick_button) {
						x = .148;
					}
					else if (gamepad1.y && !sauce) {
						x1 = x;
						sauce = true;
					}
					else if (gamepad1.y && sauce) {
						x = x1;
					}



					if (gamepad1.a && gamepad1.left_bumper) {
						robot.tapeExtend.setPower(.2);
					}
					else if (gamepad1.a && !gamepad1.left_bumper) {
						robot.tapeExtend.setPower(1);
					}
					else if (gamepad1.b && gamepad1.left_bumper){
						robot.tapeExtend.setPower(-.2);
					}
					else if (gamepad1.b && !gamepad1.left_bumper) {
						robot.tapeExtend.setPower(-1);
					}
					else {
						robot.tapeExtend.setPower(0);
					}
				}



				robot.tapeRotate.setPosition(x);
				//robot.tapeUpDown.setPosition(y);

				if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1) {
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

				if (gamepad2.dpad_left && !gamepad2.left_bumper) {
					robot.duckextend.setPower(.95);
				}
				else if (gamepad2.dpad_right && !gamepad2.left_bumper) {
					robot.duckextend.setPower(-.95);
				}
				else {
					robot.duckextend.setPower(0);
				}

				if (gamepad2.x && !gamepad2.left_bumper) {
					robot.lightsaber.setPosition(0.48);
				}
				else {
					robot.lightsaber.setPosition(.06);
				}

				if(gamepad1.x && gamepad1.left_bumper){
					big = false;
				}

				if (gamepad1.x && !gamepad1.left_bumper) {
					robot.capper.setPosition(.4);
					big = true;
				}
				else if (!big) {
					robot.capper.setPosition(.02);
				}
				else {
					robot.capper.setPosition(.22);
				}

				if (gamepad2.y) {
					cl.startWheel(speed);
				}
				else {
					cl.stopWheel(6.5);
				}

				if (gamepad1.right_trigger == 1) {
					robot.intakemotor.setPower(-.95);
				}
				else if (gamepad1.left_trigger == 1) {
					robot.intakemotor.setPower(.95);
				}
				else {
					robot.intakemotor.setPower(0);
				}

				if (gamepad1.dpad_left && !gamepad1.right_bumper && !gamepad1.left_bumper) {
					if (x <= .62) {
						x += .01;
					}
				} else if (gamepad1.dpad_right && !gamepad1.right_bumper && !gamepad1.left_bumper) {
					if (x >= 0) {
						x -= .01;
					}
				} else if (gamepad1.dpad_left && gamepad1.left_bumper) {
					if (x <= .62) {
						x += .002;
					}
				} else if (gamepad1.dpad_right && gamepad1.left_bumper) {
					if (x >= 0) {
						x -= .002;
					}
				} else if (gamepad1.dpad_left && gamepad1.right_bumper) {
					if (x <= .62) {
						x += .025;
					}
				} else if (gamepad1.dpad_right && gamepad1.right_bumper) {
					if (x >= 0) {
						x -= .025;
					}
				}

				if (gamepad1.dpad_up) {
					robot.tapeUpDown2.setPower(.85);
				}
				else if (gamepad1.dpad_down) {
					robot.tapeUpDown2.setPower(-.85);
				}
				else if (gamepad1.dpad_up && gamepad1.left_bumper) {
					robot.tapeUpDown2.setPower(.5);
				}
				else if (gamepad1.dpad_down && gamepad1.left_bumper) {
					robot.tapeUpDown2.setPower(-.5);
				}
				else {
					robot.tapeUpDown2.setPower(.02);
				}

				if (gamepad1.right_stick_button) {
					x = .42;
				}
				else if (gamepad1.left_stick_button) {
					x = .148;
				}
				else if (gamepad1.y && !sauce) {
					x1 = x;
					sauce = true;
				}
				else if (gamepad1.y && sauce) {
					x = x1;
				}

				if (gamepad1.a && gamepad1.left_bumper) {
					robot.tapeExtend.setPower(.2);
				}
				else if (gamepad1.a && !gamepad1.left_bumper) {
					robot.tapeExtend.setPower(1);
				}
				else if (gamepad1.b && gamepad1.left_bumper){
					robot.tapeExtend.setPower(-.2);
				}
				else if (gamepad1.b && !gamepad1.left_bumper) {
					robot.tapeExtend.setPower(-1);
				}
				else {
					robot.tapeExtend.setPower(0);
				}

				robot.tapeRotate.setPosition(x);

			}

			if (runtime.seconds() > 80 && runtime.seconds() < 85) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
			}
			else if (runtime.seconds() >= 85 && runtime.seconds() < 90) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
			}
			else if (runtime.seconds() >= 110 && runtime.seconds() < 115) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
			}
			else if (runtime.seconds() >= 115 && runtime.seconds() < 120) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
			}

			if (robot.blocksensor.argb() < 0) {
				if (lightsabertime.seconds() > 1) {
					closed = true;
				}
			}

			if (closed) {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
				robot.lightsaber.setPosition(.2);
			}
			else {
				robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
			}

			telemetry.addData("Status", "Run Time: " + runtime.toString());

			telemetry.addData("leftWheel", robot.frontRightMotor.getCurrentPosition());
			telemetry.addData("rightWheel", robot.rearLeftMotor.getCurrentPosition());
			telemetry.addData("middleWheel", robot.frontLeftMotor.getCurrentPosition());

			telemetry.addData("BS-ARGB", robot.blocksensor.argb());
			telemetry.addData("Closed", closed);

			telemetry.addData("colorDistance", robot.blocksensor_distance.getDistance(DistanceUnit.INCH));
			telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
			telemetry.addData("lifterTP", robot.lifter.getTargetPosition());
			telemetry.addData("Lift Power", robot.lifter.getPower());

			telemetry.addData("Duck Extend", robot.duckextend.getCurrentPosition());
			telemetry.addData("Duck Spin", robot.duckspin.getPower());

			telemetry.addData("Tape Extend Position", robot.tapeEncoder.getCurrentPosition());
			telemetry.addData("Tape Power", robot.tapeExtend.getPower());
			telemetry.addData("Tape Rotate X", x);
			telemetry.addData("Tape UpDown", robot.tapeUpDown2.getPower());

			telemetry.addData("Capper", robot.capper.getPosition());


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