package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "New Red Duckside", group = "AUTONOMOUS")
public class newRedDuckside extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
	private static final String VUFORIA_KEY = "AU2ne7j/////AAABmVnObR/UmkTwlIWslw0M3PhbCKZz1zqnqAPh50b1cKYgW7S2e0sM2P06SmDa+ClCAUh/TLJic+MN9jlOEQi+yW7ytjhnEtMyCBxMktuYhoog8VM7HpWYejdoyWu+KDbPd7820Tt16jfZGxCdiBTdvueekVz1zL2U3oPWSBDM4vdtlXE+l+wreA+SCpqeKvk7TvAgo7mk2HcqV6TZ5oB6HeTlYUhjds+x2mZ/7G0hLiEgXZlpcpP8uPAow5H1wci/0H6yx1sTylMPUGBiGQhpOBaKEmVwWZLwk/Zggfissqu3qUGXH84menZWlPv5IMDWSiBmLtoTxx4VVv/env9+v2LS0C8LiD/P+c3msMiLTM1E";
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;
	private List<Recognition> updatedRecognitions;
	private int route = 0;
	private int buffer = 3000;

	public void runOpMode() {
		initVuforia();
		initTfod();
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		Trajectory driveToDuck = drive.trajectoryBuilder(PoseLibrary.startRedDuck)
				.lineToLinearHeading(PoseLibrary.duckRed)
				.build();

		Trajectory r1AlignToScore = drive.trajectoryBuilder(driveToDuck.end())
				.splineToConstantHeading(PoseLibrary.redGoalAllianceV, Math.toRadians(270))
				.build();

		Trajectory r2Align1 = drive.trajectoryBuilder(r1AlignToScore.end())
				.splineToConstantHeading(PoseLibrary.redAwayFromElements, Math.toRadians(0))
				.splineToConstantHeading(PoseLibrary.redGoalParkingV, Math.toRadians(0))
				.build();

		Trajectory r2Align2 = drive.trajectoryBuilder(r2Align1.end())
				.lineToLinearHeading(PoseLibrary.redGoalParking)
				.build();

		Trajectory gart = drive.trajectoryBuilder(driveToDuck.end())
				.lineToLinearHeading(PoseLibrary.redGoalAlliance)
				.build();

		Trajectory gart2 = drive.trajectoryBuilder(gart.end())
				.lineToLinearHeading(PoseLibrary.redGoalAlliance)
				.build();

		while(!opModeIsActive()){
			boolean reading = false;

			if (tfod != null){

				updatedRecognitions = tfod.getUpdatedRecognitions();

				if (updatedRecognitions != null) {

					telemetry.addData("# Object Detected", updatedRecognitions.size());

					int i = 0;

					for (Recognition recognition : updatedRecognitions) {
						telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
						telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
								recognition.getLeft(), recognition.getTop());
						telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
								recognition.getRight(), recognition.getBottom());
						telemetry.addData(String.format("  width,height (%d)", i), "%.03f , %.03f",
								recognition.getWidth(), recognition.getHeight());
						telemetry.addData("Width", recognition.getImageWidth());
						telemetry.addData("Height", recognition.getImageHeight());

						i++;

						if (recognition.getLabel().equals("Ball") || recognition.getLabel().equals("Marker")){
							if(recognition.getLeft() < 250){
								telemetry.addLine("It on da left");
							}
							else{
								telemetry.addLine("Its on the right");
							}
							reading = true;
						}

					}
					String str = "";
					if(reading){
						str = "not seen";
					}
					telemetry.addLine(str);
					telemetry.update();
				}
			}

			if(gamepad1.a){
				route = 1;
			}


			if(isStopRequested()){
				break;
			}
		}

		waitForStart();
		runtime.reset();

		/* 		Steps For Duckside
				Read >150  > 400 else far Right
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

			boolean[] arr = read();
			int layer = robot.read(true, arr[0],arr[1]);
			sleep(buffer);

			drive.followTrajectory(driveToDuck);
			sleep(buffer);

			extend();
			sleep(buffer);
			spin();
			sleep(buffer);
			unextend();
			sleep(buffer);
			robot.raiseToLayer(layer);


			if(route == 0){
				drive.followTrajectory(r1AlignToScore);
				sleep(buffer);
			}
			if(route == 1){
				drive.followTrajectory(r2Align1);
				sleep(buffer);
				drive.followTrajectory(r2Align2);
				sleep(buffer);
			}

			robot.lightsaber.setPosition(robot.open);
			sleep(buffer);

			if(route == 0){

			}


			break;
		}
	}

	private boolean[] read(){
		boolean[] result = new boolean[2];

		updatedRecognitions = tfod.getUpdatedRecognitions();

		if(updatedRecognitions.size() == 0){
			result[0] = false;
			result[1] = false;
		}

		if(updatedRecognitions.get(0).getLeft() < 250){
			result[0] = true;
			result[1] = false;
		}
		else{
			result[0] = false;
			result[1] = true;
		}

		return result;
	}

	private void spin() {
		robot.duckspin.setTargetPosition(514);
		robot.duckspin.setPower(0.5);

		while(robot.duckspin.getCurrentPosition() < robot.duckspin.getTargetPosition()){
		sleep(1);
		}
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

	private void collect(SampleMecanumDrive drive){}

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

	private void initVuforia() {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Something Cool");
		parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.8f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
	}
}