package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "DEBUGGER", group = "Linear Opmode")
public class DEBUG extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();




	public void runOpMode() {


		telemetry.addData("Say", "Hello Iron Gears");
		telemetry.update();


		waitForStart();
		runtime.reset();
		robot.init(hardwareMap, this);

		float hsvValues[] = {0F, 0F, 0F};

		final float values[] = hsvValues;

		while (opModeIsActive()) {

			if(gamepad1.a)
			{
				robot.frontLeftMotor.setPower(0.4);
			}
			else{
				robot.frontLeftMotor.setPower(0);
			}

			if(isStopRequested()){
				break;
			}
		}
	}
}
