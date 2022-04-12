package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "New Red Duckside", group = "AUTONOMOUS")
public class newRedDuckside extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();
	private ElapsedTime spinTime = new ElapsedTime();

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
	private static final String VUFORIA_KEY = "AU2ne7j/////AAABmVnObR/UmkTwlIWslw0M3PhbCKZz1zqnqAPh50b1cKYgW7S2e0sM2P06SmDa+ClCAUh/TLJic+MN9jlOEQi+yW7ytjhnEtMyCBxMktuYhoog8VM7HpWYejdoyWu+KDbPd7820Tt16jfZGxCdiBTdvueekVz1zL2U3oPWSBDM4vdtlXE+l+wreA+SCpqeKvk7TvAgo7mk2HcqV6TZ5oB6HeTlYUhjds+x2mZ/7G0hLiEgXZlpcpP8uPAow5H1wci/0H6yx1sTylMPUGBiGQhpOBaKEmVwWZLwk/Zggfissqu3qUGXH84menZWlPv5IMDWSiBmLtoTxx4VVv/env9+v2LS0C8LiD/P+c3msMiLTM1E";
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;
	private List<Recognition> updatedRecognitions;

	private WebcamName intake, elevatorCamera;
	private SwitchableCamera switchableCamera;

	private int route = 0;
	private int buffer = 1000;

	public void runOpMode() {
		telemetry.addLine("step 0");
		telemetry.update();
		int step = 0;

		initVuforia();
		initTfod();
		if(tfod != null){
			tfod.activate();
			tfod.setZoom(1.25, 4.0 / 3.0);
		}

			SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
			robot.init(hardwareMap, this);
			drive.setPoseEstimate(PoseLibrary.startRedDuck);
			step++;
			telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory driveToDuck = drive.trajectoryBuilder(PoseLibrary.startRedDuck)
					.lineToLinearHeading(PoseLibrary.duckRed)
					.build();


		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

			Trajectory duckPose = drive.trajectoryBuilder(PoseLibrary.startRedDuck)
					.lineToLinearHeading(PoseLibrary.duckRed)
					.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

			Trajectory route0_1 = drive.trajectoryBuilder(duckPose.end())
					.lineToLinearHeading(PoseLibrary.redOutOfWay)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory route0_2 = drive.trajectoryBuilder(route0_1.end())
					.lineToLinearHeading(PoseLibrary.redGoalParking)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory route0_3 = drive.trajectoryBuilder(route0_2.end())
					.lineToLinearHeading(PoseLibrary.redParking0)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

		Trajectory route0_4 = drive.trajectoryBuilder(route0_3.end())
				.lineToLinearHeading(PoseLibrary.redParking0)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

			Trajectory route1_1 = drive.trajectoryBuilder(duckPose.end())
					.lineToLinearHeading(PoseLibrary.redGoalAlliance)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory route1_2 = drive.trajectoryBuilder(route1_1.end())
					.lineToLinearHeading(PoseLibrary.redWarehouseOut)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory route1_3 = drive.trajectoryBuilder(route1_2.end())
					.lineToLinearHeading(PoseLibrary.redWarehouseIn)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory route1_4a = drive.trajectoryBuilder(route1_3.end())
					.lineToLinearHeading(PoseLibrary.redParking1)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory route1_4b = drive.trajectoryBuilder(route1_3.end())
					.lineToLinearHeading(PoseLibrary.redParking2)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

			Trajectory route1_4c = drive.trajectoryBuilder(route1_3.end())
					.lineToLinearHeading(PoseLibrary.redParking3)
					.build();
		step++;
		telemetry.addLine("step " + step);
			telemetry.update();

		Trajectory loopy0_0a = drive.trajectoryBuilder(duckPose.end())
				.lineToLinearHeading(PoseLibrary.redBehindElement1)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_0b = drive.trajectoryBuilder(duckPose.end())
				.lineToLinearHeading(PoseLibrary.redBehindElement2)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_0c = drive.trajectoryBuilder(duckPose.end())
				.lineToLinearHeading(PoseLibrary.redBehindElement3)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_1a = drive.trajectoryBuilder(loopy0_0a.end())
				.lineToLinearHeading(PoseLibrary.redGoalOpposite)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_1b = drive.trajectoryBuilder(loopy0_0b.end())
				.lineToLinearHeading(PoseLibrary.redGoalOpposite)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_1c = drive.trajectoryBuilder(loopy0_0c.end())
				.lineToLinearHeading(PoseLibrary.redGoalOpposite)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_2 = drive.trajectoryBuilder(loopy0_1c.end())
				.lineToLinearHeading(PoseLibrary.redLoopBreak)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_3 = drive.trajectoryBuilder(loopy0_2.end())
				.lineToLinearHeading(PoseLibrary.redLoopBreak)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();







		String str = "";
		String str2 = "parking only";
		String str3 = "fastest and farthest";
		int parking = 0;
		telemetry.addLine("Everything is initialized!");
		TelemetryPacket t = new TelemetryPacket();
		t.addLine("Everything is initialized");
		FtcDashboard.getInstance().sendTelemetryPacket(t);
		telemetry.update();
		boolean[] arr = new boolean[2];
		arr[0] = false;
		arr[1] = false;

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

						if (recognition.getLabel().equals("Ball")) {
							if (recognition.getLeft() < 250) {
								str = "left";
							} else {
								str = "middle";
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
					str = "right";
				}
			}

			if(gamepad1.a){
				str2 = "parking only";
			}
			if(gamepad1.b){
				str2 = "warehouse";
			}
			if(gamepad1.x){
				str2 = "loopy";
			}
			if(gamepad1.dpad_left){
				str3 = "last second";
			}
			if(gamepad1.dpad_right){
				str3 = "fastest and farthest";
			}

			arr = scan();

			telemetry.addData("Index 1", arr[0]);
			telemetry.addData("Index 2", arr[1]);

			telemetry.addData("Super Path", str2 + " style, " + str3 + " parking, while avoiding " + str);
			telemetry.addLine("Use Buttons on controller 1 to change path" +
					"\n A  -  parking only" +
					"\n B  -  Warehouse" +
					"\n X  -  Looping" +
					"\n\nUse D PAD on controller 1 to change parking styles" +
					"\n LEFT\t-\tLast Second Parking" +
					"\n RIGHT\t-\tPark ASAP");

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
			t.addLine("Path Followed: " + str2 + " style, " + str3 + " parking, while avoiding " + str);
			FtcDashboard.getInstance().sendTelemetryPacket(t);

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

			if(str2.equals("parking only")) {
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
				drive.followTrajectory(route0_4);
				sleep(buffer);
			}

			if(str2.equals("warehouse")){
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
			}

			if(str2.equals("loopy")){
				if(str.equals("left")){
					drive.followTrajectory(loopy0_0a);
					sleep(buffer);
					robot.raiseToLayer(layer);
					robot.duckextend.setPower(0);
					drive.followTrajectory(loopy0_1a);
					sleep(buffer);
				}
				if(str.equals("middle")){
					drive.followTrajectory(loopy0_0b);
					sleep(buffer);
					robot.raiseToLayer(layer);
					robot.duckextend.setPower(0);
					drive.followTrajectory(loopy0_1b);
					sleep(buffer);
				}
				if(str.equals("right")){
					drive.followTrajectory(loopy0_0c);
					sleep(buffer);
					robot.raiseToLayer(layer);
					robot.duckextend.setPower(0);
					drive.followTrajectory(loopy0_1c);
					sleep(buffer);
				}
				robot.lightsaber.setPosition(robot.open);
				sleep(buffer);
				drive.followTrajectory(loopy0_2);
				sleep(buffer);
				robot.raiseToLayer(0);
				sleep(buffer);
			}


			break;
		}
	}

	private boolean[] scan() {
		boolean[] result = new boolean[2];
		result[0] = false;
		result[1] = false;

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

		robot.duckspin.setTargetPosition(-514);
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

		intake = hardwareMap.get(WebcamName.class, "Something Cool");
		elevatorCamera = hardwareMap.get(WebcamName.class, "Something Awesome");
		parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(intake, elevatorCamera);
		parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		switchableCamera = (SwitchableCamera) vuforia.getCamera();
		switchableCamera.setActiveCamera(elevatorCamera);

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