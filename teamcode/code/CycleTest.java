package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Collect", group = "Linear Opmode")
public class CycleTest extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		double pi = Math.PI;
		double tile = 24;
		int parking = 0;
		robot.distancearmservo1.setPosition(robot.retreating);
		robot.distancearmservo2.setPosition(robot.retreating);
		robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.elementarm.setPosition(0);
		while(!opModeIsActive()){
			if(gamepad1.a){

			}
			telemetry.addData("Parking Position", posToWord(parking));
			telemetry.update();
		}

		waitForStart();
		runtime.reset();

		Pose2d starty = new Pose2d(6,-67,Math.toRadians(270));
		Pose2d ducky = new Pose2d(-41, -60, Math.toRadians(270));
		Pose2d redParking0 = new Pose2d(-60, -32, Math.toRadians(0));
		Pose2d redE1 = new Pose2d(-44.5, -48, Math.toRadians(270));
		Pose2d redE2 = new Pose2d(-36, -48, Math.toRadians(270));
		Pose2d redE3 = new Pose2d(-27.5, -48, Math.toRadians(270));
		Pose2d redE4 = new Pose2d(3.5, -48, Math.toRadians(270));
		Pose2d redE5 = new Pose2d(12, -48, Math.toRadians(270));
		Pose2d redE6 = new Pose2d(20.5, -48, Math.toRadians(270));
		Pose2d redGoalAlliance = new Pose2d(-12, -36, Math.toRadians(270));
		Pose2d redGoalAlliance45 = new Pose2d(0, -36, Math.toRadians(315));
		Pose2d redGoalAlliancePrep = new Pose2d(-12, -40, Math.toRadians(270));
		Pose2d redGoalParking = new Pose2d(-36, -24, Math.toRadians(180));
		Pose2d redGoalWarehouse = new Pose2d(12, -24, Math.toRadians(0));
		Pose2d redParking1 = new Pose2d(36, -65, Math.toRadians(0));
		Pose2d redParking2 = new Pose2d(36, -36, Math.toRadians(315));
		Pose2d redParking3 = new Pose2d(60, -36, Math.toRadians(270));
		Pose2d redWarehouseOut = new Pose2d(12, -65, Math.toRadians(0));
		Pose2d redWarehouseIn = new Pose2d(40, -65, Math.toRadians(0));
		Pose2d Reading = null;
		Pose2d E_Ending = null;

		drive.setPoseEstimate(redWarehouseIn);

		double DR = 0;
		double DL = 0;
		int layer = 0;
		int targetL = 0;
		int duckTarget = 300;



		while (opModeIsActive()) {

			//trajectories

			Trajectory moveFromWall = drive.trajectoryBuilder(starty)
					.lineToLinearHeading(redGoalAlliance)
					.build();

			Trajectory fromGoalToEntrance = drive.trajectoryBuilder(redGoalAlliance)
					.lineToLinearHeading(redWarehouseOut)
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

			Trajectory score = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(redGoalAlliance)
					.build();

			Trajectory score45 = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(redGoalAlliance45)
					.build();

			//--------------------------------------------------------------------------------------
			int constant = 1;
			constant = collect(constant,drive);
			robot.intakemotor.setPower(0.95);

			Trajectory leave = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(redWarehouseOut)
					.build();

			drive.followTrajectory(leave);
			robot.intakemotor.setPower(0);

			break;
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

		robot.intakemotor.setPower(-0.95);

		drive.followTrajectory(eliminateconstant);

		if(robot.distance3.getDistance(DistanceUnit.INCH) < ttb || robot.blocksensor_distance.getDistance(DistanceUnit.INCH) < 3){
			robot.intakemotor.setPower(0.95);
			return constant;
		}

		while(robot.distance3.getDistance(DistanceUnit.INCH) > 3 && robot.blocksensor_distance.getDistance(DistanceUnit.INCH) > 3){
			robot.intakemotor.setPower(-0.95);

		Trajectory forwardByThree = drive.trajectoryBuilder(drive.getPoseEstimate())
				.forward(3,
						SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(5))
				.build();

			drive.followTrajectory(forwardByThree);

			constant+=3;

			telemetry.addData("constant", constant);
			telemetry.addData("reading", robot.distance3.getDistance(DistanceUnit.INCH));
			telemetry.addData("reading lightsaber", robot.blocksensor_distance.getDistance(DistanceUnit.INCH));
			telemetry.update();

			sleep(1000);

			if(isStopRequested()){
				break;
			}
		}

		return constant;
	}


	private void liftIt() {
		robot.lifter.setTargetPosition(robot.layer3A);
		robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.lifter.setPower(0.85);

	}

	private void errorCorrection(SampleMecanumDrive drive) {
		Trajectory errorCorrection = drive.trajectoryBuilder(drive.getPoseEstimate())
				.lineToLinearHeading(new Pose2d(
								drive.getPoseEstimate().getX()-drive.getLastError().getX(),
								drive.getPoseEstimate().getY()-drive.getLastError().getY()),
						SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(5))
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