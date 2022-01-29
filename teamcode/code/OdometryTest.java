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

@Autonomous(name = "Odometry Test", group = "Linear Opmode")
public class OdometryTest extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();


	double frontleft = 0;
	double rearleft = 0;
	double rearright = 0;
	double frontright = 0;

	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		waitForStart();
		runtime.reset();

		Pose2d starty = new Pose2d(0,0,Math.toRadians(90));


		while (opModeIsActive()) {

			Trajectory step1 = drive.trajectoryBuilder(new Pose2d())
					.strafeTo(new Vector2d(-36,-36))
					.build();

			Trajectory step2 = drive.trajectoryBuilder(step1.end())
					.strafeTo(new Vector2d(15,15))
					.build();

			Trajectory step3 = drive.trajectoryBuilder(step2.end())
					.strafeTo(new Vector2d(0,-30))
					.build();

			Trajectory step4 = drive.trajectoryBuilder(step3.end())
					.strafeTo(new Vector2d(-15,15))
					.build();

			drive.followTrajectory(step1);
			sleep(2500);

			drive.followTrajectory(step2);
			sleep(2500);

			drive.followTrajectory(step3);
			sleep(2500);

			drive.followTrajectory(step4);
			sleep(2500);

			break;
		}
	}
}