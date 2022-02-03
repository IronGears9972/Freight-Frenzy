package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "DRIVE CHECK", group = "Linear Opmode")
public class AutoDriveSequence extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		double pi = Math.PI;
		double tile = 24;

		waitForStart();
		runtime.reset();

		Pose2d starty = new Pose2d(-41,-65,Math.toRadians(270));
		Pose2d redParking0 = new Pose2d(-60, -32, Math.toRadians(0));
		Pose2d redE1 = new Pose2d(-44.5, -43.25, Math.toRadians(270));
		Pose2d redE2 = new Pose2d(-36, -43.25, Math.toRadians(270));
		Pose2d redE3 = new Pose2d(-27.5, -43.25, Math.toRadians(270));
		Pose2d redE4 = new Pose2d(3.5, -43.25, Math.toRadians(270));
		Pose2d redE5 = new Pose2d(12, -43.25, Math.toRadians(270));
		Pose2d redE6 = new Pose2d(20.5, -43.25, Math.toRadians(270));
		Pose2d redGoalAlliance = new Pose2d(-12, -36, Math.toRadians(270));
		Pose2d redGoalParking = new Pose2d(-36, -24, Math.toRadians(180));
		Pose2d redGoalWarehouse = new Pose2d(12, -24, Math.toRadians(0));
		Pose2d redParking1 = new Pose2d(39, -64, Math.toRadians(0));
		Pose2d redParking2 = new Pose2d(39, -39, Math.toRadians(315));
		Pose2d redParking3 = new Pose2d(64, -39, Math.toRadians(270));
		Pose2d redWarehouseOut = new Pose2d(12, -64, Math.toRadians(0));
		Pose2d redWarehouseIn = new Pose2d(36, -64, Math.toRadians(0));
		Pose2d outOfWay = new Pose2d(-12,-60,Math.toRadians(270));



		drive.setPoseEstimate(redWarehouseOut);

		while (opModeIsActive()) {

			//Trajectories HERE


			Trajectory step1 = drive.trajectoryBuilder(redWarehouseOut)
					.lineToLinearHeading(redParking1)
					.build();
			Trajectory step2 = drive.trajectoryBuilder(step1.end())
					.lineToLinearHeading(redParking2)
					.build();
			Trajectory step3 = drive.trajectoryBuilder(step2.end())
					.lineToLinearHeading(redParking3)
					.build();
			Trajectory errorCorrection = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(new Pose2d(
							drive.getPoseEstimate().getX()+drive.getLastError().getX(),
							drive.getPoseEstimate().getY()+drive.getLastError().getY()))
					.build();

			drive.followTrajectory(step1);
			sleep(250);
			drive.followTrajectory(step2);
			sleep(250);
			drive.followTrajectory(step3);
			sleep(250);


			break;
		}
	}
}