package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Splining testing", group = "AUTONOMOUS")
public class AutonomousCycle extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		double pi = Math.PI;
		double tile = 24;
		int parking = 0;
		robot.distancearmservo1.setPosition(robot.reading);
		robot.distancearmservo2.setPosition(robot.reading+0.01);
		while(!opModeIsActive()) {

		}

		waitForStart();
		runtime.reset();

		Pose2d starty = new Pose2d(6,-66,Math.toRadians(270));
		Pose2d ducky = new Pose2d(-41, -60, Math.toRadians(270));
		Pose2d away = new Pose2d(0, -48, Math.toRadians(270));
		Pose2d redParking0 = new Pose2d(-60, -32, Math.toRadians(0));
		Pose2d redE1 = new Pose2d(-44.5, -48, Math.toRadians(270));
		Pose2d redE2 = new Pose2d(-36, -48, Math.toRadians(270));
		Pose2d redE3 = new Pose2d(-27.5, -48, Math.toRadians(270));
		Pose2d redE4 = new Pose2d(3.5, -48, Math.toRadians(270));
		Pose2d redE5 = new Pose2d(12, -48, Math.toRadians(270));
		Pose2d redE6 = new Pose2d(20.5, -48, Math.toRadians(270));
		Pose2d redGoalAlliance = new Pose2d(-12, -38, Math.toRadians(270)); //mega fixed
		Pose2d redGoalAlliance45 = new Pose2d(-1, -33, Math.toRadians(315)); //fixed
		Pose2d redGoalAlliance45Prep = new Pose2d(0, -32, Math.toRadians(315));
		Pose2d redGoalAlliancePrep = new Pose2d(-12, -40, Math.toRadians(270));
		Pose2d redGoalParking = new Pose2d(-36, -24, Math.toRadians(180));
		Pose2d redGoalWarehouse = new Pose2d(12, -24, Math.toRadians(0));
		Pose2d redParking1 = new Pose2d(36, -65, Math.toRadians(0));
		Pose2d redParking2 = new Pose2d(36, -36, Math.toRadians(315));
		Pose2d redParking3 = new Pose2d(60, -36, Math.toRadians(270));
		Pose2d redWarehouseOut = new Pose2d(12, -65, Math.toRadians(0));
		Pose2d redWarehouseIn = new Pose2d(42, -65, Math.toRadians(0));
		Pose2d Reading = null;
		Pose2d E_Ending = null;

		drive.setPoseEstimate(starty);

		double DR = 0;
		double DL = 0;
		int layer = 0;
		int targetL = 0;
		int duckTarget = 300;
		int testSleep = 0;

		while (opModeIsActive()) {

			Trajectory fart = drive.trajectoryBuilder(starty)
					.lineToLinearHeading(redGoalAlliance)
					.build();


			break;
		}
	}


}