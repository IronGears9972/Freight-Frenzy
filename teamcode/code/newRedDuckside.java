package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "New Red Duckside", group = "ARED")
public class newRedDuckside extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();
	private ElapsedTime spinTime = new ElapsedTime();
	private ElapsedTime returnTime = new ElapsedTime();

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
	private static final String VUFORIA_KEY = "AU2ne7j/////AAABmVnObR/UmkTwlIWslw0M3PhbCKZz1zqnqAPh50b1cKYgW7S2e0sM2P06SmDa+ClCAUh/TLJic+MN9jlOEQi+yW7ytjhnEtMyCBxMktuYhoog8VM7HpWYejdoyWu+KDbPd7820Tt16jfZGxCdiBTdvueekVz1zL2U3oPWSBDM4vdtlXE+l+wreA+SCpqeKvk7TvAgo7mk2HcqV6TZ5oB6HeTlYUhjds+x2mZ/7G0hLiEgXZlpcpP8uPAow5H1wci/0H6yx1sTylMPUGBiGQhpOBaKEmVwWZLwk/Zggfissqu3qUGXH84menZWlPv5IMDWSiBmLtoTxx4VVv/env9+v2LS0C8LiD/P+c3msMiLTM1E";
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;
	private List<Recognition> updatedRecognitions;

	private WebcamName intake, elevatorCamera;
	private SwitchableCamera switchableCamera;

	private int route = 0;
	private int buffer = 500;
	int x = 0;

	public void runOpMode() {
		telemetry.addLine("step 0");
		telemetry.update();
		int step = 0;

		initVuforia();
		initTfod();
		if(tfod != null){
			tfod.activate();
			tfod.setZoom(1.5, 4.0 / 2.4);
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
				.lineToLinearHeading(PoseLibrary.duckRed,
						SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(15))
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory ParkPath_Step1 = drive.trajectoryBuilder(duckPose.end())
				.lineToLinearHeading(PoseLibrary.redOutOfWay,
						SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(30))
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory ParkPath_DriveToScore = drive.trajectoryBuilder(ParkPath_Step1.end())
				.lineToLinearHeading(PoseLibrary.redGoalParking,
						SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(30))
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory ParkPath_Reverse = drive.trajectoryBuilder(ParkPath_DriveToScore.end())
				.lineToLinearHeading(PoseLibrary.redOutOfWay)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();


		Trajectory ParkPath_PickDuck = drive.trajectoryBuilder(ParkPath_Reverse.end())
				.lineToLinearHeading(new Pose2d(-48,-48,Math.toRadians(270)))
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();


		Trajectory ParkPath_Parking = drive.trajectoryBuilder(ParkPath_Reverse.end())
				.lineToLinearHeading(PoseLibrary.redParking0)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_DriveToScore = drive.trajectoryBuilder(duckPose.end())
				.lineToLinearHeading(PoseLibrary.redGoalAlliance,
						SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(30))
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_DriveToEntrance = drive.trajectoryBuilder(WarehousePath_DriveToScore.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseOut)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_EnterWarehouse = drive.trajectoryBuilder(WarehousePath_DriveToEntrance.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseIn)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_ParkFar1 = drive.trajectoryBuilder(WarehousePath_EnterWarehouse.end())
				.lineToLinearHeading(PoseLibrary.redParking2)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_ParkFar2 = drive.trajectoryBuilder(WarehousePath_ParkFar1.end())
				.lineToLinearHeading(PoseLibrary.redParking3)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_WaitHere = drive.trajectoryBuilder(WarehousePath_DriveToScore.end())
				.lineToLinearHeading(PoseLibrary.redPause)
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_DriveToEntrance2 = drive.trajectoryBuilder(WarehousePath_WaitHere.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseOut3,
						SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(30))
				.build();
		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory WarehousePath_EnterAndPark = drive.trajectoryBuilder(WarehousePath_DriveToEntrance2.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseIn)
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
				.lineToLinearHeading(PoseLibrary.redWarehouseOut)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_3b = drive.trajectoryBuilder(loopy0_2.end())
				.lineToLinearHeading(PoseLibrary.redManeuverAvoidMiddle)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_4b = drive.trajectoryBuilder(loopy0_3b.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseOut)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();

		Trajectory loopy0_5 = drive.trajectoryBuilder(loopy0_4b.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseIn)
				.build();

		step++;
		telemetry.addLine("step " + step);
		telemetry.update();


		String str = "";
		String str2 = "alliance parking";
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
			robot.lightsaber.setPosition(robot.lightsaber45);

			if (tfod != null) {


				updatedRecognitions = tfod.getUpdatedRecognitions();

				if (updatedRecognitions != null) {
					if(updatedRecognitions.size() != 0){

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


						if (recognition.getLabel().equals("Ball")) {
							arr = scan();
							x = i;
							if (recognition.getLeft() < 250) {
								str = "left";
							} else {
								str = "middle";
							}
							i++;
						}
					}


				} else {
					str = "right";
					arr[0] = false;
					arr[1] = false;
				}
					telemetry.addData("Raw Left 1", arr[0]);
					telemetry.addData("Raw Right 2", arr[1]);

					telemetry.addData("\nSuper Path", "\n" +
							"Route: " + str2 +
							"\nParking: " + str3 +
							"\nElement: " + str);
					telemetry.addLine("\nUse Buttons on controller 1 to change path" +
							"\n A  -  Alliance Parking" +
							"\n B  -  Warehouse" +
							"\n X  -  Looping" +
							"\n\nUse D PAD on controller 1 to change parking styles" +
							"\n LEFT\t-\tLast Second Parking" +
							"\n RIGHT\t-\tPark Far and Fast");
					telemetry.update();
			}
			}

			if(gamepad1.a){
				str2 = "alliance parking";
			}
			if(gamepad1.b){
				str2 = "warehouse";
			}
			if(gamepad1.x){
				str2 = "loopy";
			}
			if(gamepad1.y){
				str2 = "pick up";
			}
			if(gamepad1.dpad_left){
				str3 = "last second";
			}
			if(gamepad1.dpad_right){
				str3 = "fastest and farthest";
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
			t.addLine("Path Followed: " + str2 + " style, " + str3 + " parking, while avoiding " + str);
			FtcDashboard.getInstance().sendTelemetryPacket(t);
			returnTime.reset();

			int layer = robot.read(true, arr[0],arr[1]);
			sleep(buffer);

			drive.followTrajectory(duckPose);
			sleep(buffer);

			extend();
			sleep(buffer);

			spin();
			sleep(buffer+2000);

			unextend();
			sleep(buffer);

			if(str2.equals("alliance parking") || str2.equals("pick up")) {
				drive.followTrajectory(ParkPath_Step1);
				sleep(buffer);
				robot.raiseToLayer(layer);
				robot.duckextend.setPower(0);
				drive.followTrajectory(ParkPath_DriveToScore);
				sleep(buffer);
				robot.lightsaber.setPosition(robot.lightsaberKick);
				sleep(buffer);
				robot.raiseToLayer(0);
				drive.followTrajectory(ParkPath_Reverse);
				sleep(buffer);
				if(str2.equals("alliance parking")) {
					drive.followTrajectory(ParkPath_Parking);
					sleep(buffer);
				}
				else{
					drive.followTrajectory(ParkPath_PickDuck);
					sleep(buffer);
				}
			}

			if(str2.equals("warehouse")){
				robot.raiseToLayer(layer);
				drive.followTrajectory(WarehousePath_DriveToScore);
				sleep(buffer);
				robot.lightsaber.setPosition(robot.lightsaberKick);
				sleep(buffer);
				robot.duckextend.setPower(0);
				if(str3.equals("fastest and farthest")) {
					drive.followTrajectory(WarehousePath_DriveToEntrance);
					sleep(buffer);
					robot.raiseToLayer(0);
					drive.followTrajectory(WarehousePath_EnterWarehouse);
					sleep(buffer);
					drive.followTrajectory(WarehousePath_ParkFar1);
					sleep(buffer);
					drive.followTrajectory(WarehousePath_ParkFar2);
					sleep(buffer);
				}
				if(str3.equals("last second")){
					drive.followTrajectory(WarehousePath_WaitHere);
					sleep(buffer);
					robot.raiseToLayer(0);

					while(!timeLeft(drive)){
						telemetry.addData("Current Time", returnTime.seconds());
						telemetry.update();
						sleep(10);
					}

					drive.followTrajectory(WarehousePath_DriveToEntrance2);
					sleep(buffer);


					drive.followTrajectory(WarehousePath_EnterAndPark);
					sleep(buffer);
				}
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
				robot.lightsaber.setPosition(robot.lightsaberKick);
				sleep(buffer);
				drive.followTrajectory(loopy0_2);
				sleep(buffer);
				robot.raiseToLayer(0);
				sleep(buffer);
				if(!str.equals("middle")){
					drive.followTrajectory(loopy0_3);
					sleep(buffer);
				}
				else{
					drive.followTrajectory(loopy0_3b);
					sleep(buffer);
					drive.followTrajectory(loopy0_4b);
					sleep(buffer);
				}

				if(str3.equals("last second")){
					while(!timeLeft(drive)){
						telemetry.addData("Current Time", returnTime.seconds());
						telemetry.update();
						sleep(10);
					}
				}

				drive.followTrajectory(loopy0_5);
			}


			telemetry.addLine("Path Followed: " + str2 + " style, " + str3 + " parking, while avoiding " + str);
			telemetry.update();
			break;
		}
	}

	private boolean[] scan() {
		boolean[] result = new boolean[2];
		result[0] = false;
		result[1] = false;

		if (updatedRecognitions != null) {

			if (updatedRecognitions.get(x).getLeft() < 250) {
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

		robot.duckspin.setTargetPosition(-764);
		robot.duckspin.setPower(0.90);

		while(robot.duckspin.getCurrentPosition() < robot.duckspin.getTargetPosition() && spinTime.seconds() < 15){
			robot.duckspin.setPower(0.90);
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

		while(robot.duckextend.getCurrentPosition() < robot.duckextend.getTargetPosition() && guy.seconds() < 15){

			robot.duckextend.setPower(0.85);
			telemetry.addData("CP",robot.duckextend.getCurrentPosition());
			telemetry.addData("TP",robot.duckTarget);
			telemetry.update();
			sleep(1);
		}

		robot.duckextend.setPower(0);


	}

	private Pose2d findFreight(Recognition recognition, SampleMecanumDrive drive){

		double roboX = drive.getPoseEstimate().getX();
		double roboY = drive.getPoseEstimate().getY();
		double roboHead = drive.getPoseEstimate().getHeading();

		double camX = roboX + 5.2137*Math.cos(roboHead);
		double camY = roboY + 0.6685*Math.sin(roboHead);
		double camHead = roboHead;

		double displacement = getDisplacement(recognition);
		double midpoint = getMidpoint(recognition);
		double shift = getLeftRight(displacement,midpoint);
		double hypotenuse = Math.sqrt(Math.pow(displacement,2) + Math.pow(shift,2));

		//this works because look at the engineering notebook
		Pose2d recognizedHere = new Pose2d(
				camX + hypotenuse*Math.cos(camHead + Math.atan(shift/displacement)),
				camY + hypotenuse*Math.sin(camHead + Math.atan(shift/displacement))
		);

		return recognizedHere;
	}


	private double getDisplacement(Recognition recognition){

		/*
		 * we have two methods to figure out distance, distance in pixels from an edge of the top/bottom of screen, and width
		 * As these move further away from the camera, width decreases, and pixels from top/bottom
		 *
		 */
		double width = recognition.getWidth();
		double displacement1 = -0.1121*recognition.getWidth() + 34.58;
		double displacement2 = 0.02601*(recognition.getImageHeight()-recognition.getTop()) - 1.518;
		/*
				y = ax^2 + bx + c
				a: 1.045
				b: -33.03
				c: 394.2
		*/
		double d3 = 0.001009*width - 0.4418*width + 55.63;

		return d3;

	}

	private double getMidpoint(Recognition recognition){
		//returns center y axis of the recognition
		return recognition.getLeft() + (recognition.getWidth()/2);
	}

	private double getLeftRight(double displacement, double midpoint){
		/*
		 * half range is equal to the half the range that the camera can read based on an input distance. this is because we know FOV is 55deg
		 * pixelsMaybe(name not finalized) is turning the range in inches into a pixel measurement. We want this because the midpoint is in pixels (640.0 is derived from camera's output range)
		 * rawResult is an unfinalized number. by dividing midpoint over pixelsMaybe(name not finalized) we will get the midpoint in inches. We subtract the half range FOR SOME REASON
		 * the return statement gives back a more condensed number (a tolerance of 0.001 inches is not scary)
		 */
		double halfRange = displacement*Math.tan(Math.toRadians(27.5));
		double pixelsmaybe = 640.0/(2*halfRange);
		double rawResult = (midpoint/pixelsmaybe) - halfRange;
		return rawResult - (rawResult%0.001);
	}

	private boolean timeLeft(SampleMecanumDrive drive){

		/*
			distance = sqrt((x2-x1)^2 + (y2-y1)^2)

			distance/maxvelocity = time it takes to get to a point

			if time away is greater than time remaining then break out and leave for the parking
		 */

		double X0 = drive.getPoseEstimate().getX();
		double Y0 = drive.getPoseEstimate().getY();
		double X1 = PoseLibrary.redWarehouseOut.getX();
		double Y1 = PoseLibrary.redWarehouseOut.getY();
		double distance = Math.sqrt( Math.pow( (X0-X1) , 2 )  +  Math.pow( (Y0-Y1) , 2 )  );
		double timeAway = distance/60.0;
		double timeLeft = 25 - returnTime.seconds();

		if(timeAway > timeLeft){
			return true;
		}
		else{
			return false;
		}
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
		tfodParameters.minResultConfidence = 0.68f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
		FtcDashboard.getInstance().startCameraStream(tfod,0);
	}
}