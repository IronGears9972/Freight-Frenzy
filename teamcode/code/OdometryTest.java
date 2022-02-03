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

@Autonomous(name = "ODOMETRY TEST", group = "Linear Opmode")
public class OdometryTest extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		boolean duckside = true;
		while (!opModeIsActive()){
			if (gamepad1.a){
				duckside = true;
			}
			else if (gamepad1.b){
				duckside = false;
			}
			telemetry.addData("Parking Pos.", duckside);
			telemetry.update();
			if (isStopRequested()){
				break;
			}
		}

		waitForStart();
		runtime.reset();

		Pose2d starty = new Pose2d(-41,-65,Math.toRadians(270));
		drive.setPoseEstimate(starty);
		drive.getLastError();
		drive.getPoseEstimate();



		while (opModeIsActive()) {

			//driving steps HERE
			Trajectory step2 = drive.trajectoryBuilder(starty)
					.lineToLinearHeading(new Pose2d(-13.75,-37.79,Math.toRadians(270)))
					.build();

			Trajectory step3 = drive.trajectoryBuilder(step2.end())
					.lineToLinearHeading(new Pose2d(-24,-48,Math.toRadians(270)))
					.build();

			Trajectory step4 = drive.trajectoryBuilder(step3.end())
					.lineToLinearHeading(new Pose2d(-48,-48,Math.toRadians(270)))
					.build();

			//Trajectory errorCorrection = drive.trajectoryBuilder(step4.end())

			Trajectory step5 = drive.trajectoryBuilder(step4.end())
					.lineToLinearHeading(new Pose2d(-61,-36.787,Math.toRadians(270)))
					.build();

			drive.followTrajectory(step2);
			sleep(50);

			drive.followTrajectory(step3);
			sleep(50);

			drive.followTrajectory(step4);
			sleep(50);

			drive.followTrajectory(step5);
			sleep(50);


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