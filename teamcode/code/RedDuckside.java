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

@Autonomous(name = "Red Duckside", group = "Linear Opmode")
public class RedDuckside extends LinearOpMode {

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
		while(!opModeIsActive()){
			if(gamepad1.a){

			}
			telemetry.addData("Parking Position", posToWord(parking));
			telemetry.update();
		}

		waitForStart();
		runtime.reset();

		Pose2d starty = 			new Pose2d(-41,-65,Math.toRadians(270));
		Pose2d ducky = 				new Pose2d(-41, -60, Math.toRadians(270));
		Pose2d redParking0 = 		new Pose2d(-60, -36, Math.toRadians(0));
		Pose2d redE1 = 				new Pose2d(-44.5, -49, Math.toRadians(270));
		Pose2d redE2 = 				new Pose2d(-36, -49, Math.toRadians(270));
		Pose2d redE3 = 				new Pose2d(-27.5, -48.5, Math.toRadians(270));
		Pose2d redE4 =			 	new Pose2d(3.5, -48, Math.toRadians(270));
		Pose2d redE5 =				new Pose2d(12, -48, Math.toRadians(270));
		Pose2d redE6 = 				new Pose2d(20.5, -48, Math.toRadians(270));
		Pose2d redGoalAlliance = 	new Pose2d(-12, -36, Math.toRadians(270));
		Pose2d redGoalParking = 	new Pose2d(-27, -24, Math.toRadians(180));
		Pose2d redGoalWarehouse = 	new Pose2d(12, -24, Math.toRadians(0));
		Pose2d redParking1 = 		new Pose2d(36, -65, Math.toRadians(0));
		Pose2d redParking2 = 		new Pose2d(36, -36, Math.toRadians(315));
		Pose2d redParking3 = 		new Pose2d(60, -36, Math.toRadians(270));
		Pose2d redWarehouseOut = 	new Pose2d(12, -65, Math.toRadians(0));
		Pose2d redWarehouseIn = 	new Pose2d(36, -65, Math.toRadians(0));
		Pose2d Reading = null;


		Pose2d E_Ending = null;

		drive.setPoseEstimate(starty);

		double DR = 0;
		double DL = 0;
		int layer = 0;
		int targetL = 0;
		int duckTarget = -1180;



		while (opModeIsActive()) {

			//driving steps HERE

			Trajectory align = null;
			Trajectory startToDuckPos = drive.trajectoryBuilder(starty)
					.lineToLinearHeading(ducky)
					.build();

			//reads the element's position on the field

			for (int x = 0; x < 10; ++x) {
				DR = DR + robot.distance2.getDistance(DistanceUnit.INCH);
				DL = DL + robot.distance1.getDistance(DistanceUnit.INCH);
				telemetry.addData("DR", DR);
				telemetry.addData("DL", DL);
				telemetry.update();
				sleep(10);
			}

			DR = DR / 10;
			DL = DL / 10;

			if (DL > 17 && DL < 24) {
				layer = 1;
			} else if (DR > 17 && DR < 24) {
				layer = 2;
			} else {
				layer = 3;
			}
			sleep(250);
			robot.distancearmservo1.setPosition(robot.retreating);
			robot.distancearmservo2.setPosition(robot.retreating);

			//duck extend and spin it

			drive.followTrajectory(startToDuckPos);
			sleep(250);
			robot.duckextend.setTargetPosition(duckTarget);
			robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			while(robot.duckextend.getCurrentPosition() > duckTarget){
				robot.duckextend.setPower(0.95);
				telemetry.addData("CP", robot.duckextend.getCurrentPosition());
				telemetry.addData("TP", duckTarget);
				telemetry.update();
				sleep(1);
			}
			robot.duckextend.setPower(0);
			sleep(250);
			accSpin(1500);
			robot.duckspin.setPower(0);
			robot.duckextend.setTargetPosition(0);
			robot.duckextend.setPower(-0.8);


			robot.elementarm.setPosition(robot.down);
			robot.elementclamp1.setPosition(robot.open);
			robot.elementclamp2.setPosition(robot.open);
			sleep(2500);

			if (layer == 1){
				//GO TO LEFT ELEMENT
				align = drive.trajectoryBuilder(ducky)
						.lineToLinearHeading(redE1)
						.build();
				targetL = robot.layer1A;
			}
			else if (layer == 2){
				//GO TO MIDDLE ELEMENT
				align = drive.trajectoryBuilder(ducky)
						.lineToLinearHeading(redE2)
						.build();
				targetL = robot.layer2A;
			}
			else if (layer == 3){
				//GO TO RIGHT ELEMENT
				align = drive.trajectoryBuilder(ducky)
						.lineToLinearHeading(redE3)
						.build();
				targetL = robot.layer3A;
			}

			drive.followTrajectory(align);

			Trajectory grabSlow = drive.trajectoryBuilder(align.end())
					.back(2.5,
							SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
							SampleMecanumDrive.getAccelerationConstraint(5))
					.build();

			drive.followTrajectory(grabSlow);

			sleep(500);

			robot.elementclamp1.setPosition(robot.closed);
			robot.elementclamp2.setPosition(robot.closed);
			sleep(500);
			robot.elementarm.setPosition(robot.up);

			if(parking == 0){
				//go to drop on the parking side, and then break
				Trajectory parkingGoal = drive.trajectoryBuilder(align.end())
						.lineToLinearHeading(redGoalParking)
						.build();

				raise(targetL);
				sleep(500);

				drive.followTrajectory(parkingGoal);
				sleep(250);
				robot.lightsaber.setPosition(robot.open);
				sleep(500);

				Trajectory prepPark = drive.trajectoryBuilder(redGoalParking)
						.lineToLinearHeading(new Pose2d(-36,-24,Math.toRadians(180)))
						.build();

				drive.followTrajectory(prepPark);

				sleep(250);
				dropIt();

				Trajectory parkInGoal = drive.trajectoryBuilder(prepPark.end())
						.lineToLinearHeading(redParking0)
						.build();

				drive.followTrajectory(parkInGoal);



			}

			//strafe to align left and right distance sensor

			teleOpReset();
			break;
		}
	}

	private void accSpin(int i) {
		double increment = 0;
		for(int x = 0; x < 10; x++){
			robot.duckspin.setPower(-(0.75 + increment));
			sleep(i/10);
			increment += 0.025;
		}

	}
	private void dropIt() {
		robot.lifter.setTargetPosition(0);
		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lifter.setPower(0.85);
	}

	private void accSpin(int i, int y) {
		double increment = 0;
		for(int x = 0; x < 10; x++){
			robot.duckspin.setPower(-(0.75 + increment));
			sleep(i/10);
			increment += 0.025;
		}

	}

	private void teleOpReset() {
		robot.lightsaber.setPosition(1);
		robot.lifter.setTargetPosition(0);
		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		while(robot.lifter.getCurrentPosition()>0){
			robot.lifter.setPower(0.65);
			telemetry.addData("Current-Lift", robot.lifter.getCurrentPosition());
			telemetry.addData("Target-Lift", 0);
			telemetry.update();
			sleep(1);
		}
		robot.lifter.setPower(0);
		sleep(250);
	}

	private void raise(int level) {
		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lifter.setTargetPosition(level);
		while(robot.lifter.getCurrentPosition()<level){
			robot.lifter.setPower(0.65);
			telemetry.addData("Current-Lift", robot.lifter.getCurrentPosition());
			telemetry.addData("Target-Lift", level);
			telemetry.update();
			sleep(1);
		}
		robot.lifter.setPower(0);
		sleep(250);



	}

	private String posToWord(int park) {
		String result = "";

		if (park == 0){
			result = "Alliance Specific Warehouse";
		}
		if (park == 1){
			result = "Warehouse Entry (Near the Tiered Goal)";
		}
		if (park == 2){
			result = "Middle of the Warehouse";
		}
		if (park == 3){
			result =  "Warehouse Exit (Near the Shared Goal)";
		}

		return result;
	}
}