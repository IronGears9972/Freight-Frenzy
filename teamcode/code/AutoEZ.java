package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "EZ POINTS", group = "Linear Opmode")
public class AutoEZ extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	boolean spot = true;

	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);


		while (!opModeIsActive()) {

			if (gamepad1.a) {
				spot = true;
			} else if (gamepad1.b) {
				spot = false;
			}

			telemetry.addData("PARKING POS.", spot);
			telemetry.update();

			if(isStopRequested()){
				break;
			}

		}

		waitForStart();
		runtime.reset();

		Pose2d starty = new Pose2d(-41,-65,Math.toRadians(270));
		drive.setPoseEstimate(starty);

		Pose2d start2 = new Pose2d(0,-65,Math.toRadians(270));
		drive.setPoseEstimate(starty);


		while (opModeIsActive()) {

			//driving steps HERE

			Trajectory step1 = drive.trajectoryBuilder(starty)
					.lineToLinearHeading(new Pose2d(-61,-36.787,Math.toRadians(270)))
					.build();

			Trajectory step2 = drive.trajectoryBuilder(start2)
					.lineToLinearHeading(new Pose2d(12,-63,Math.toRadians(360)))
					.build();

			Trajectory step3 = drive.trajectoryBuilder(step2.end())
					.lineToLinearHeading(new Pose2d(-61,-36.787,Math.toRadians(360)))
					.build();


			if(spot) {
				drive.followTrajectory(step1);
				sleep(2500);
			}
			else{
				drive.followTrajectory(step2);
				sleep(2500);

			}

			//duck extend and spin it: NOT DOING 1/29/22 - BROKEN


			//strafe to align left and right distance sensor


			//distance check algorithm


			//align with the element


			//drive to be aligned with the shipping hub


			//raise arm to drop in randomized level


			//SPLIT: Strafe to right in front of the warehouse or to alliance place


			//Set up for Teleop


			break;
		}
	}
}