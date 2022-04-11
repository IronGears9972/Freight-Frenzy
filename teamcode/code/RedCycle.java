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

@Autonomous(name = "Red Warehouse", group = "AUTONOMOUS")
public class RedCycle extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		double pi = Math.PI;
		double tile = 24;
		int parking = 0;
		while(!opModeIsActive()){
			if(gamepad1.a){
				parking = 1;
			}
			if(gamepad1.b){
				parking = 2;
			}
			if(gamepad1.x){
				parking = 3;
			}
			if(gamepad1.y){
				parking = 0;
			}
			telemetry.addData("Parking Position", posToWord(parking));
			telemetry.addLine("A - " + posToWord(1) +
					"\n B - " + posToWord(2) +
					"\n X - " + posToWord(3) +
					"\n Y - " + posToWord(0));
			telemetry.update();
			if(isStopRequested()){
				break;
			}
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

			//trajectories

			Trajectory moveFromWall = drive.trajectoryBuilder(starty)
					.lineToLinearHeading(redGoalAlliance)
					.build();



			Trajectory from45GoalToEntrance = drive.trajectoryBuilder(redGoalAlliance45)
					.lineToLinearHeading(redWarehouseOut)
					.build();

			Trajectory EnterWarehouse = drive.trajectoryBuilder(redWarehouseOut)
					.lineToLinearHeading(redWarehouseIn)
					.build();

			Trajectory ExitWarehouse = drive.trajectoryBuilder(redWarehouseIn)
					.lineToLinearHeading(redWarehouseOut)
					.build();

			Trajectory align = drive.trajectoryBuilder(redWarehouseOut)
					.lineToLinearHeading(redGoalAlliancePrep)
					.build();

			//errorCorrection(drive);

			Trajectory score = drive.trajectoryBuilder(redGoalAlliancePrep)
					.lineToLinearHeading(redGoalAlliance)
					.build();

			Trajectory score45 = drive.trajectoryBuilder(redWarehouseOut)
					.lineToLinearHeading(redGoalAlliance45)
					.build();


			//--------------------------------------------------------------------------------------


			DR = DR / 10;
			DL = DL / 10;

			if (DL > 17 && DL < 24) {
				layer = robot.layer1A;
			} else if (DR > 17 && DR < 24) {
				layer = robot.layer2A;
			} else {
				layer = robot.layer3A;
			}

			robot.lifter.setTargetPosition(layer);
			sleep(testSleep);
			robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.lifter.setPower(0.85);

			drive.followTrajectory(moveFromWall);
			sleep(testSleep);

			robot.lightsaber.setPosition(robot.open);

			sleep(testSleep);

			Trajectory unalign2 = drive.trajectoryBuilder(drive.getPoseEstimate())
					.strafeTo(new Vector2d(0,-48))
					.build();

			drive.followTrajectory(unalign2);
			dropIt();
			sleep(testSleep);

			int collectionIncrement = 6;
			robot.lightsaber.setPosition(1);
			Trajectory fromStartGoalToEntrance = drive.trajectoryBuilder(unalign2.end())
					.lineToLinearHeading(redWarehouseOut)
					.build();

			drive.followTrajectory(fromStartGoalToEntrance);
			sleep(testSleep);


			drive.followTrajectory(EnterWarehouse);
			sleep(testSleep);


			collectionIncrement = collect(collectionIncrement,drive);

			robot.intakemotor.setPower(0.95);
			sleep(testSleep);

			Trajectory leave1 = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(redWarehouseIn)
					.build();

			drive.followTrajectory(leave1);
			robot.intakemotor.setPower(0);

			Trajectory leave2 = drive.trajectoryBuilder(leave1.end())
					.lineToLinearHeading(redWarehouseOut)
					.build();

			drive.followTrajectory(leave2);

			liftIt();

			sleep(testSleep);

			if(layer == robot.layer1A){
				drive.followTrajectory(align);
				sleep(testSleep);
				drive.followTrajectory(score);
				sleep(testSleep);
			}
			else {
				drive.followTrajectory(score45);
				sleep(testSleep);
			}

			robot.lightsaber.setPosition(robot.open);
			sleep(550);

			Trajectory unalign = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(away)
					.build();

			drive.followTrajectory(unalign);
			robot.lightsaber.setPosition(1);

			dropIt();
			sleep(testSleep);

			Trajectory fromGoalToEntrance = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(redWarehouseOut)
					.build();

			drive.followTrajectory(fromGoalToEntrance);
			sleep(testSleep);

			drive.followTrajectory(EnterWarehouse);
			sleep(testSleep);

			Trajectory changeUp = drive.trajectoryBuilder(EnterWarehouse.end())
					.strafeLeft(18)
					.build();

			drive.followTrajectory(changeUp);

			collect(collectionIncrement,drive);

			robot.intakemotor.setPower(0.95);

			park(parking,drive,redParking1,redParking2,redParking3);



			break;
		}
	}

	private void park(int parking, SampleMecanumDrive drive,Pose2d pose1,Pose2d pose2,Pose2d pose3) {
		Trajectory park2 = drive.trajectoryBuilder(drive.getPoseEstimate())
				.lineToLinearHeading(pose2)
				.build();
		Trajectory park1 = drive.trajectoryBuilder(park2.end())
				.lineToLinearHeading(pose1)
				.build();
		Trajectory park3 = drive.trajectoryBuilder(park2.end())
				.lineToLinearHeading(pose3)
				.build();
		if(parking == 1){
			drive.followTrajectory(park2);
			sleep(250);
			drive.followTrajectory(park1);
		}
		if(parking == 2){
			drive.followTrajectory(park2);
		}
		if(parking == 3){
			drive.followTrajectory(park2);
			sleep(250);
			drive.followTrajectory(park3);
		}
	}

	private void dropIt() {
		robot.lifter.setTargetPosition(0);
		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lifter.setPower(0.85);
	}

	private int collect(int constant, SampleMecanumDrive drive){
		//drives forward and collects an object
		boolean reading = false;
		double ttb = 3;
		int totalDisplacement = constant;

		Trajectory eliminateconstant = drive.trajectoryBuilder(drive.getPoseEstimate())
				.forward(constant)
				.build();

		robot.intakemotor.setPower(-0.9);

		drive.followTrajectory(eliminateconstant);

		if(robot.blocksensor_distance.getDistance(DistanceUnit.INCH) < 3){
			robot.intakemotor.setPower(0.95);
			return constant;
		}

		while(robot.blocksensor_distance.getDistance(DistanceUnit.INCH) > 3){
			robot.intakemotor.setPower(-0.9);

			Trajectory forwardByThree = drive.trajectoryBuilder(drive.getPoseEstimate())
					.forward(2,
							SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
							SampleMecanumDrive.getAccelerationConstraint(15))
					.build();

			drive.followTrajectory(forwardByThree);

			constant+=2;

			telemetry.addData("constant", constant);
			telemetry.addData("reading lightsaber", robot.blocksensor_distance.getDistance(DistanceUnit.INCH));
			telemetry.update();

			sleep(100);

			if(isStopRequested()){
				break;
			}
		}

		return constant;
	}

	private int collect2(int constant, double power){

		while(robot.blocksensor_distance.getDistance(DistanceUnit.INCH) > 3){
			robot.frontLeftMotor.setPower(power);
			robot.frontRightMotor.setPower(power);
			robot.rearLeftMotor.setPower(power);
			robot.rearRightMotor.setPower(power);


		}

		return constant;
	}

	private void liftIt() {
		robot.lifter.setTargetPosition(robot.layer3A);
		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lifter.setPower(0.85);

	}

	private void liftIt(int layer) {
		if(layer == 1){
			robot.lifter.setTargetPosition(robot.layer1A);
		}
		else if (layer == 2){
			robot.lifter.setTargetPosition(robot.layer2A);
		}
		else if (layer == 3){
			robot.lifter.setTargetPosition(robot.layer3A);
		}

		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lifter.setPower(0.85);

	}

	private void errorCorrection(SampleMecanumDrive drive) {
		Trajectory errorCorrection = drive.trajectoryBuilder(drive.getPoseEstimate())
				.lineToLinearHeading(new Pose2d(
								drive.getPoseEstimate().getX()-drive.getLastError().getX(),
								drive.getPoseEstimate().getY()-drive.getLastError().getY()),
						SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(10))
				.build();
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