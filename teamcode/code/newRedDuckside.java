package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "New Red Duckside", group = "AUTONOMOUS")
public class newRedDuckside extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();


	public void runOpMode() {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		while(!opModeIsActive()){

			Trajectory driveToDuck = drive.trajectoryBuilder(PoseLibrary.startRedDuck)
					.lineToLinearHeading(PoseLibrary.duckRed)
					.build();

			Trajectory gart2 = drive.trajectoryBuilder(driveToDuck.end())
					.splineToConstantHeading(PoseLibrary.redGoalAllianceV, Math.toRadians(270))
					.build();

			Trajectory gart3 = drive.trajectoryBuilder(gart2.end())
					.splineToConstantHeading(PoseLibrary.redAwayFromElements, Math.toRadians(270))
					.splineToConstantHeading(PoseLibrary.redGoalAllianceV, Math.toRadians(270))
					.build();

			if(isStopRequested()){
				break;
			}
		}

		waitForStart();
		runtime.reset();

		/* 		Steps For Duckside
				Read
				Spin
				NO PICK UP SO AVOID ELEMENT
				Raise
				Score
				Reverse and lower
				Park in a spot
					- in baby zone
					- in warehouse
					- last second mode
		 */



		while (opModeIsActive()) {

			int layer = robot.read(true,true,false);
			//drive to duck spinner
			extend();
			spin();
			unextend();
			robot.raiseToLayer(layer);
			//drive to score
			robot.lightsaber.setPosition(robot.open);
			//park



			break;
		}
	}

	private void spin() {
	}

	private void unextend() {
		
	}

	public void extend(){

		robot.duckextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.duckextend.setTargetPosition(robot.duckTarget);
		robot.duckextend.setPower(0.95);

		while(robot.duckextend.getCurrentPosition() < robot.duckTarget){
			telemetry.addData("CP",robot.duckextend.getCurrentPosition());
			telemetry.addData("TP",robot.duckTarget);
			telemetry.update();
			sleep(1);
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

	private void collect(SampleMecanumDrive drive){
		//drives forward and collects an object

		Trajectory eliminateconstant = drive.trajectoryBuilder(drive.getPoseEstimate())
				.forward(12)
				.build();

		robot.intakemotor.setPower(-0.9);

		drive.followTrajectory(eliminateconstant);

		if(robot.distance3.getDistance(DistanceUnit.INCH) < 3 || robot.blocksensor_distance.getDistance(DistanceUnit.INCH) < 3){
			robot.intakemotor.setPower(0.95);
		}

		while(robot.distance3.getDistance(DistanceUnit.INCH) > 3 && robot.blocksensor_distance.getDistance(DistanceUnit.INCH) > 3){
			robot.intakemotor.setPower(-0.9);

			Trajectory forwardByThree = drive.trajectoryBuilder(drive.getPoseEstimate())
					.forward(2,
							SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
							SampleMecanumDrive.getAccelerationConstraint(10))
					.build();

			drive.followTrajectory(forwardByThree);

			telemetry.addData("reading", robot.distance3.getDistance(DistanceUnit.INCH));
			telemetry.addData("reading lightsaber", robot.blocksensor_distance.getDistance(DistanceUnit.INCH));
			telemetry.update();

			sleep(100);

			if(isStopRequested()){
				break;
			}
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
			result = "Warehouse Entry LAST SECOND";
		}
		if (park == 2){
			result = "Pre-load in the Warehouse";
		}
		if (park == 3){
			result =  "Warehouse Exit (Near the Shared Goal)";
		}

		return result;
	}
}