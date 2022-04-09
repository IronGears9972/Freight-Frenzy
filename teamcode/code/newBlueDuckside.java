package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name = "New blue Duckside", group = "AUTONOMOUS")
public class newBlueDuckside extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();
	private ElapsedTime spinTime = new ElapsedTime();

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
	private static final String VUFORIA_KEY = "AU2ne7j/////AAABmVnObR/UmkTwlIWslw0M3PhbCKZz1zqnqAPh50b1cKYgW7S2e0sM2P06SmDa+ClCAUh/TLJic+MN9jlOEQi+yW7ytjhnEtMyCBxMktuYhoog8VM7HpWYejdoyWu+KDbPd7820Tt16jfZGxCdiBTdvueekVz1zL2U3oPWSBDM4vdtlXE+l+wreA+SCpqeKvk7TvAgo7mk2HcqV6TZ5oB6HeTlYUhjds+x2mZ/7G0hLiEgXZlpcpP8uPAow5H1wci/0H6yx1sTylMPUGBiGQhpOBaKEmVwWZLwk/Zggfissqu3qUGXH84menZWlPv5IMDWSiBmLtoTxx4VVv/env9+v2LS0C8LiD/P+c3msMiLTM1E";
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;
	private List<Recognition> updatedRecognitions;
	private int route = 0;
	private int buffer = 3000;

	public void runOpMode() {
		telemetry.addLine("step 0");
		telemetry.update();

		initVuforia();
		initTfod();
		if(tfod != null){
			tfod.activate();
			tfod.setZoom(1.25, 4.0 / 3.0);
		}
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);
		drive.setPoseEstimate(PoseLibrary.startblueDuck);

		telemetry.addLine("step 1");
		telemetry.update();

		Trajectory driveToDuck = drive.trajectoryBuilder(PoseLibrary.startblueDuck)
				.lineToLinearHeading(PoseLibrary.duckblue)
				.build();

		telemetry.addLine("step 2");
		telemetry.update();

		Trajectory duckPose = drive.trajectoryBuilder(PoseLibrary.startblueDuck)
				.lineToLinearHeading(PoseLibrary.duckblue)
				.build();
		telemetry.addLine("step 3");
		telemetry.update();

		Trajectory route0_1 = drive.trajectoryBuilder(duckPose.end())
				.lineToLinearHeading(PoseLibrary.blueOutOfWay)
				.build();
		telemetry.addLine("step 4");
		telemetry.update();

		Trajectory route0_2 = drive.trajectoryBuilder(route0_1.end())
				.lineToLinearHeading(PoseLibrary.blueGoalParking)
				.build();
		telemetry.addLine("step 5");
		telemetry.update();

		Trajectory route0_3 = drive.trajectoryBuilder(route0_2.end())
				.lineToLinearHeading(PoseLibrary.blueParking0)
				.build();
		telemetry.addLine("step 6");
		telemetry.update();

		Trajectory route1_1 = drive.trajectoryBuilder(duckPose.end())
				.lineToLinearHeading(PoseLibrary.blueGoalAlliance)
				.build();
		telemetry.addLine("step 7");
		telemetry.update();

		Trajectory route1_2 = drive.trajectoryBuilder(route1_1.end())
				.lineToLinearHeading(PoseLibrary.blueWarehouseOut)
				.build();
		telemetry.addLine("step 8");
		telemetry.update();

		Trajectory route1_3 = drive.trajectoryBuilder(route1_2.end())
				.lineToLinearHeading(PoseLibrary.blueWarehouseIn)
				.build();
		telemetry.addLine("step 9");
		telemetry.update();

		Trajectory route1_4a = drive.trajectoryBuilder(route1_3.end())
				.lineToLinearHeading(PoseLibrary.blueParking1)
				.build();
		telemetry.addLine("step 10");
		telemetry.update();

		Trajectory route1_4b = drive.trajectoryBuilder(route1_3.end())
				.lineToLinearHeading(PoseLibrary.blueParking2)
				.build();
		telemetry.addLine("step 11");
		telemetry.update();

		Trajectory route1_4c = drive.trajectoryBuilder(route1_3.end())
				.lineToLinearHeading(PoseLibrary.blueParking3)
				.build();
		telemetry.addLine("step 12");
		telemetry.update();





		String str = "";
		int parking = 0;
		telemetry.addLine("Everything is initialized!");
		telemetry.update();

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

						if (recognition.getLabel().equals("Ball") || recognition.getLabel().equals("Marker")) {
							if (recognition.getLeft() < 250) {
								str = "Its on da left";
							} else {
								str = "It on the right";
							}
						}
					}

					telemetry.addLine(str);
					telemetry.addData("route", posToWord(route));
					telemetry.addLine("A - " + posToWord(1) + "\nB - " + posToWord(2) + "\nX - " + posToWord(3) + "\nY - " + posToWord(0));
					telemetry.addLine(parking + "");
					telemetry.update();
				}
				else{
					str = "not seen";
				}
			}

			if(gamepad1.a){
				route = 1;
			}
			if(gamepad1.b){
				route = 2;
			}
			if(gamepad1.x){
				route = 3;
			}
			if(gamepad1.y){
				route = 0;
			}

			if(gamepad1.dpad_up){
				parking = 1;
			}
			if(gamepad1.dpad_right){
				parking = 2;
			}
			if(gamepad1.dpad_down){
				parking = 3;
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

			drive.followTrajectory(duckPose);
			sleep(buffer);

			extend();
			sleep(buffer);

			spin();
			sleep(buffer);

			unextend();
			sleep(buffer);

			if(route == 0) {
				drive.followTrajectory(route0_1);
				sleep(buffer);
				robot.raiseToLayer(layer);
				robot.duckextend.setPower(0);
				drive.followTrajectory(route0_2);
				sleep(buffer);
				robot.lightsaber.setPosition(robot.open);
				sleep(buffer);
				robot.raiseToLayer(0);
				drive.followTrajectory(route0_3);
				sleep(buffer);
			}

			if(route == 1){
				robot.raiseToLayer(layer);
				robot.duckextend.setPower(0);
				drive.followTrajectory(route1_1);
				sleep(buffer);
				robot.lightsaber.setPosition(robot.open);
				sleep(buffer);
				drive.followTrajectory(route1_2);
				sleep(buffer);
				robot.raiseToLayer(0);
				drive.followTrajectory(route1_3);
				sleep(buffer);
				if(parking == 1) {
					drive.followTrajectory(route1_4a);
					sleep(buffer);
				}
				if(parking == 2) {
					drive.followTrajectory(route1_4b);
					sleep(buffer);
				}
				if(parking == 3) {
					drive.followTrajectory(route1_4c);
					sleep(buffer);
				}

			}


			break;
		}
	}

	private boolean[] read() {
		boolean[] result = new boolean[2];
		result[0] = false;
		result[1] = false;

		updatedRecognitions = tfod.getUpdatedRecognitions();

		if (updatedRecognitions != null) {

			if (updatedRecognitions.get(0).getLeft() < 350) {
				result[0] = true;
				result[1] = false;
			} else {
				result[0] = false;
				result[1] = true;
			}
		}
		return result;
	}


	private void spin() {
		spinTime.reset();

		robot.duckspin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.duckspin.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		robot.duckspin.setTargetPosition(514);
		robot.duckspin.setPower(0.85);

		while(robot.duckspin.getCurrentPosition() < robot.duckspin.getTargetPosition() && spinTime.seconds() < 5){
			robot.duckspin.setPower(0.85);
			telemetry.addData("CP2",robot.duckspin.getCurrentPosition());
			telemetry.addData("TP2",robot.duckspin.getTargetPosition());
			telemetry.update();
			sleep(1);
		}
	}

	private void unextend() {
		robot.duckextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.duckextend.setTargetPosition(0);
		robot.duckextend.setPower(0.95);

	}

	public void extend(){

		ElapsedTime guy = new ElapsedTime();

		robot.duckextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.duckextend.setTargetPosition(robot.duckTarget);
		robot.duckextend.setPower(0.85);

		while(robot.duckextend.getCurrentPosition() < robot.duckextend.getTargetPosition() && guy.seconds() < 5){

			robot.duckextend.setPower(0.85);
			telemetry.addData("CP",robot.duckextend.getCurrentPosition());
			telemetry.addData("TP",robot.duckTarget);
			telemetry.update();
			sleep(1);
		}

		robot.duckextend.setPower(0);


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
			result = "basic route";
		}
		if (park == 1){
			result = "Warehouse Entry LAST SECOND";
		}
		if (park == 2){
			result = "Pre-load in the Warehouse";
		}
		if (park == 3){
			result =  "Warehouse Exit (Near the Shablue Goal)";
		}

		return result;
	}

	private void initVuforia() {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Something Awesome");
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
		FtcDashboard.getInstance().startCameraStream(tfod,0);
	}
}