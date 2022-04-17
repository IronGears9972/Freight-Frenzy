package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "New Red Blandside", group = "ARED   ")
public class newRedBland extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();
	private ElapsedTime spinTime = new ElapsedTime();
	private ElapsedTime returnTime = new ElapsedTime();

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
	private static final String VUFORIA_KEY = "AU2ne7j/////AAABmVnObR/UmkTwlIWslw0M3PhbCKZz1zqnqAPh50b1cKYgW7S2e0sM2P06SmDa+ClCAUh/TLJic+MN9jlOEQi+yW7ytjhnEtMyCBxMktuYhoog8VM7HpWYejdoyWu+KDbPd7820Tt16jfZGxCdiBTdvueekVz1zL2U3oPWSBDM4vdtlXE+l+wreA+SCpqeKvk7TvAgo7mk2HcqV6TZ5oB6HeTlYUhjds+x2mZ/7G0hLiEgXZlpcpP8uPAow5H1wci/0H6yx1sTylMPUGBiGQhpOBaKEmVwWZLwk/Zggfissqu3qUGXH84menZWlPv5IMDWSiBmLtoTxx4VVv/env9+v2LS0C8LiD/P+c3msMiLTM1E";
	private VuforiaLocalizer vuforia;
	//private VuforiaLocalizer vuforia2;
	private TFObjectDetector tfod;
	private List<Recognition> updatedRecognitions;

	private WebcamName intake, elevatorCamera;
	private SwitchableCamera switchableCamera;

	private ArrayList<Pose2d> Readings = new ArrayList<Pose2d>();
	private int route = 0;
	private int buffer = 100;
	private int x = 0;
	private boolean[] arr = new boolean[2];


	public void runOpMode() {
		telemetry.addLine("step 0");
		telemetry.update();
		arr[0] = false;
		arr[1] = false;

		initVuforia();
		initTfod();
		if(tfod != null){
			tfod.activate();
			tfod.setZoom(1, 4.0/3.0);
		}
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);

		drive.setPoseEstimate(PoseLibrary.startRedBland);

		telemetry.addLine("step 1");
		telemetry.update();

		Trajectory gart = drive.trajectoryBuilder(PoseLibrary.startRedBland)
				.lineToLinearHeading(PoseLibrary.redGoalAlliance2)
				.build();

		Trajectory firstTime01 = drive.trajectoryBuilder(gart.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseOut)
				.build();

		Trajectory firstTime02 = drive.trajectoryBuilder(firstTime01.end())
				.lineToLinearHeading(PoseLibrary.redReading,
						SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(35))
				.build();

		Trajectory cycling00 = drive.trajectoryBuilder(PoseLibrary.redWarehouseIn)
				.lineToLinearHeading(PoseLibrary.redWarehouseOut)
				.build();

		Trajectory cycling01 = drive.trajectoryBuilder(cycling00.end())
				.lineToLinearHeading(PoseLibrary.redGoalAlliance,
						SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(35))
				.build();

		Trajectory cycling02 = drive.trajectoryBuilder(cycling01.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseOut)
				.build();

		Trajectory cycling03 = drive.trajectoryBuilder(cycling02.end())
				.lineToLinearHeading(PoseLibrary.redWarehouseIn)
				.build();

		Trajectory cycleIn = drive.trajectoryBuilder(PoseLibrary.redGoalAlliance)
				.splineToSplineHeading(PoseLibrary.redWarehouseOut,Math.toRadians(0),
						SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(30))
				.splineToConstantHeading(PoseLibrary.redWarehouseInV, Math.toRadians(180),
						SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(30))
				.build();


		Trajectory parkIt = drive.trajectoryBuilder(PoseLibrary.redWarehouseOut)
				.lineToLinearHeading(PoseLibrary.redParking1)
				.build();

		String str = "";
		int parking = 0;
		int layer = 0;
		telemetry.addLine("Everything is initialized!");
		telemetry.update();

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
					telemetry.update();
				}
			}

			if(isStopRequested()){
				break;
			}
		}

		waitForStart();
		runtime.reset();

		/* 		Steps For Blandside
			- read it
			- drop the pre loaded

				loop
			- enter warehouse
			- collect
			- exit warehouse
			repeat here until bad time* is left
			- park

		 */



		while (opModeIsActive()) {
			returnTime.reset();

			layer = robot.read(true, arr[0],arr[1]);

			robot.raiseToLayer(layer);

			telemetry.update();
			sleep(buffer);
			drive.followTrajectory(gart);
			sleep(buffer+750);
			robot.lightsaber.setPosition(0.4);
			sleep(buffer+500);

			switchableCamera.setActiveCamera(intake);

			//drive.followTrajectory(firstTime01);
			sleep(buffer);
			robot.raiseToLayer(0);
			drive.followTrajectory(cycleIn); //firstTime02
			//read(drive);

			while(!timeLeft(drive)){
				if(true) {
					/*
					if (Readings.size() == 0 ) {
						while (Readings.size() == 0) {
							read(drive);
						}
					}

					 */


					robot.intakemotor.setPower(-0.95);
					robot.lightsaber.setPosition(0);
					//drive.followTrajectory(collect(drive));

					if(timeLeft(drive)){
						break;
					}


					while(robot.blocksensor_distance.getDistance(DistanceUnit.INCH) > 1.5 || isStopRequested()){
						double power = 0.2;
						robot.frontLeftMotor.setPower(power);
						robot.frontRightMotor.setPower(power);
						robot.rearLeftMotor.setPower(power);
						robot.rearRightMotor.setPower(power);
						if(timeLeft(drive) || isStopRequested()){
							break;
						}
					}

					robot.lightsaber.setPosition(0.2);
					robot.intakemotor.setPower(0.95);
					double power = 0;
					robot.frontLeftMotor.setPower(power);
					robot.frontRightMotor.setPower(power);
					robot.rearLeftMotor.setPower(power);
					robot.rearRightMotor.setPower(power);

					if(timeLeft(drive)){
						break;
					}
					sleep(buffer);


					Trajectory reAlign = drive.trajectoryBuilder(drive.getPoseEstimate())
							.lineToLinearHeading(PoseLibrary.redWarehouseIn)
							.build();

					drive.followTrajectory(reAlign);
					robot.intakemotor.setPower(0);

					drive.followTrajectory(cycling00);
					if(timeLeft(drive)){
						break;
					}

					//outside below

					sleep(buffer);
					drive.followTrajectory(cycling01);
					if(timeLeft(drive)){
						break;
					}
					robot.raiseToLayer(3);
					while(robot.lifter.getCurrentPosition() < robot.lifter.getTargetPosition()-15){
						telemetry();
						if(timeLeft(drive)){
							break;
						}
						sleep(1);
					}
					if(timeLeft(drive)){
						break;
					}
					robot.lightsaber.setPosition(0.4);
					sleep(buffer+500);
					robot.raiseToLayer(0);
					//drive.followTrajectory(cycling02);
					if(timeLeft(drive)){
						break;
					}
					sleep(buffer);
					drive.followTrajectory(cycleIn);
					robot.lightsaber.setPosition(0);
					if(timeLeft(drive)){
						break;
					}
					sleep(buffer);
				}
				else{
					read(drive);
				}

			}
			Trajectory finalAlign = drive.trajectoryBuilder(drive.getPoseEstimate())
					.lineToLinearHeading(PoseLibrary.redWarehouseOut)
					.build();

			drive.followTrajectory(finalAlign);
			sleep(buffer);
			drive.followTrajectory(parkIt);
			sleep(buffer);

			break;
		}
	}

	private void read(SampleMecanumDrive drive) {
		updatedRecognitions = tfod.getUpdatedRecognitions();
		int i = 0;
		if(updatedRecognitions != null) {
			for (Recognition recognition : updatedRecognitions) {
				telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
				telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
						recognition.getLeft(), recognition.getTop());
				telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
						recognition.getRight(), recognition.getBottom());
				telemetry.addData(String.format("  width,height (%d)", i), "%.03f , %.03f",
						recognition.getWidth(), recognition.getHeight());

				telemetry.addData("width", recognition.getWidth());
				telemetry.addData("Distance", getDisplacement(recognition));
				telemetry.update();

				i++;
				if (recognition.getLabel().equals("Ball") || recognition.getLabel().equals("Cube") || recognition.getLabel().equals("Duck")) {
					Pose2d here = findFreight(recognition, drive);
					if (valid(Readings, here)) {
						Readings.add(here);
					}
				}
			}
		}
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
		double timeLeft = 58 - returnTime.seconds();

		if(timeAway > timeLeft){
			return true;
		}
		else{
			return false;
		}
	}

	private boolean[] scan() {
		boolean[] result = new boolean[2];
		result[0] = false;
		result[1] = false;

		//updatedRecognitions = tfod.getUpdatedRecognitions();

		if (updatedRecognitions.size() > 0) {

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

	private void telemetry(){
		int i = 0;
		for(Pose2d pose : Readings){
			telemetry.addLine("Reading: " + i);
			telemetry.addData("X", pose.getX());
			telemetry.addData("Y", pose.getY());

			i++;
		}
		telemetry.update();
	}

	private Trajectory collect(SampleMecanumDrive drive){

		Trajectory result = null;
		Pose2d closest = new Pose2d(1000,0,0);
		for(Pose2d pose : Readings){
			if (pose.getX() < closest.getX()){
				closest = pose;
			}
		}
		result = drive.trajectoryBuilder(drive.getPoseEstimate())
				.lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(),closest.getY()),
						SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
						SampleMecanumDrive.getAccelerationConstraint(10))
				.build();
		Readings.remove(closest);
		return result;

	}

	private boolean valid(ArrayList<Pose2d> readings, Pose2d here) {
		double X1 = here.getX();
		double Y1 = here.getY();
		for(Pose2d pose : readings){
			double distance = Math.sqrt( Math.pow( (pose.getX()-X1) , 2 )  +  Math.pow( (pose.getY()-Y1) , 2 )  );

			if(distance < 2.5){
				return false;
			}

		}
		return true;
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
		double displacement1 = -0.1121*recognition.getWidth() + 34.58;
		double displacement2 = 0.02601*(recognition.getImageHeight()-recognition.getTop()) - 1.518;

		if (displacement2 > 9){
			return (displacement1+displacement2)/2.0;
		}
		else{
			return displacement2;
		}

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
		tfodParameters.minResultConfidence = 0.5f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
		FtcDashboard.getInstance().startCameraStream(tfod,0);
	}
}