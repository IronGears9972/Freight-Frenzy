package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "DEBUGGER", group = "ZDEBUG")
public class DEBUG extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
	private static final String VUFORIA_KEY = "AU2ne7j/////AAABmVnObR/UmkTwlIWslw0M3PhbCKZz1zqnqAPh50b1cKYgW7S2e0sM2P06SmDa+ClCAUh/TLJic+MN9jlOEQi+yW7ytjhnEtMyCBxMktuYhoog8VM7HpWYejdoyWu+KDbPd7820Tt16jfZGxCdiBTdvueekVz1zL2U3oPWSBDM4vdtlXE+l+wreA+SCpqeKvk7TvAgo7mk2HcqV6TZ5oB6HeTlYUhjds+x2mZ/7G0hLiEgXZlpcpP8uPAow5H1wci/0H6yx1sTylMPUGBiGQhpOBaKEmVwWZLwk/Zggfissqu3qUGXH84menZWlPv5IMDWSiBmLtoTxx4VVv/env9+v2LS0C8LiD/P+c3msMiLTM1E";
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;
	private List<Recognition> updatedRecognitions;

	private WebcamName intake, elevatorCamera;
	private SwitchableCamera switchableCamera;

	String test = "";
	char ins = '\u00B7';

	public void runOpMode() {

		ArrayList<Integer>BallReadingA = new ArrayList<>();
		ArrayList<Integer>BallReadingR = new ArrayList<>();
		ArrayList<Integer>BallReadingG = new ArrayList<>();
		ArrayList<Integer>BallReadingB = new ArrayList<>();
		ArrayList<Integer>BallReadingARGB = new ArrayList<>();
		ArrayList<Integer>blockReadingA = new ArrayList<>();
		ArrayList<Integer>blockReadingR = new ArrayList<>();
		ArrayList<Integer>blockReadingG = new ArrayList<>();
		ArrayList<Integer>blockReadingB = new ArrayList<>();
		ArrayList<Integer>blockReadingARGB = new ArrayList<>();

		while (!opModeIsActive()) {

			if (gamepad1.a) {
				test = "Duck Extend / Spin";
			}
			else if (gamepad1.b) {
				test = "Elevator";
			}
			else if (gamepad1.x) {
				test = "Wheels";
			}
			else if (gamepad1.y) {
				test = "Distances + Color";
			}
			else if (gamepad1.dpad_up) {
				test = "Elevator Auto";
			}
			else if (gamepad1.dpad_down) {
				test = "Tape";
			}
			else if (gamepad1.dpad_right) {
				test = "Aspect Ratio";
			}

			telemetry.addData("Testing", test);
			telemetry.addLine("Press buttons to change what is tested\n" +
					"\t"+ ins + "\tA - Duck Extender and Spinner\n" +
					"\t"+ ins + "\tB - Elevator\n" +
					"\t"+ ins + "\tY - Distances and Color Sensors\n" +
					"\t"+ ins + "\tX - Wheels\n" +
					"\t"+ ins + "\tDPAD_UP - Elevator\n" +
					"\t"+ ins + "\tDPAD_Down - Tape Crap\n" +
					"\t"+ ins + "\tDPAD_Right - Tape Crap");
			telemetry.update();

			if(isStopRequested()){
				break;
			}

		}
		initVuforia();
		initTfod();

		double num = 4.0;
		double denom = 3.0;
		double mag = 1.0;

		if(tfod != null){
			tfod.activate();
			tfod.setZoom(mag, num / denom);
		}

		waitForStart();
		runtime.reset();
		robot.init(hardwareMap, this);
		int stored1 = 0;
		int stored2 = 0;
		int stored3 = 0;
		int stored4 = 0;
		int max = Integer.MIN_VALUE;
		int min = Integer.MAX_VALUE;
		int sumBaA = 0;
		int sumBaR = 0;
		int sumBaG = 0;
		int sumBaB = 0;
		int sumBaARGB = 0;
		int sumBlA = 0;
		int sumBlR = 0;
		int sumBlG = 0;
		int sumBlB = 0;
		int sumBlARGB = 0;
		double y = 0.0;
		double x = 0.5;

		while (opModeIsActive()) {

			robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
			robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);

			if(test.equals("Aspect Ratio")){
				tfod.setZoom(mag, num / denom);

				if(gamepad1.left_stick_x != 0){
					num += 0.005* gamepad1.left_stick_x;
				}
				if(gamepad1.right_stick_x != 0){
					denom += 0.005* gamepad1.right_stick_x;
				}
				if(gamepad1.right_trigger != 0){
					mag += 0.005* gamepad1.right_trigger;
				}
				if(gamepad1.left_trigger != 0 && mag >= 1.01){
					mag -= 0.005* gamepad1.left_trigger;
				}

				if(gamepad1.left_bumper && gamepad1.right_bumper){
					mag = 1;
					denom = 3.0;
					num = 4.0;
				}
				telemetry.addData("mag", mag);
				telemetry.addData("num", num);
				telemetry.addData("denom", denom);
				telemetry.update();

			}

			if(test.equals("Tape")){
				if(gamepad1.a){
					y = 0.9;
				}
				else if(gamepad1.b){
					y = -0.9;
				}
				else{
					y = 0;
				}

				if (gamepad1.dpad_left && !gamepad1.right_bumper && !gamepad1.left_bumper) {
					if (x <= .45) {
						x += .01;
					}
				} else if (gamepad1.dpad_right && !gamepad1.right_bumper && !gamepad1.left_bumper) {
					if (x >= 0) {
						x -= .01;
					}
				} else if (gamepad1.dpad_left && gamepad1.left_bumper) {
					if (x <= .45) {
						x += .002;
					}
				} else if (gamepad1.dpad_right && gamepad1.left_bumper) {
					if (x >= 0) {
						x -= .002;
					}
				} else if (gamepad1.dpad_left && gamepad1.right_bumper) {
					if (x <= .45) {
						x += .025;
					}
				} else if (gamepad1.dpad_right && gamepad1.right_bumper) {
					if (x >= 0) {
						x -= .025;
					}
				}

				if (gamepad1.left_trigger > 0.6 && gamepad1.left_bumper) {
					robot.tapeExtend.setPower(.2);
				}
				else if (gamepad1.left_trigger > 0.6 && !gamepad1.left_bumper) {
					robot.tapeExtend.setPower(1);
				}
				else if (gamepad1.left_trigger > 0.6 && gamepad1.left_bumper){
					robot.tapeExtend.setPower(-.2);
				}
				else if (gamepad1.right_trigger > 0.6 && !gamepad1.left_bumper) {
					robot.tapeExtend.setPower(-1);
				}
				else {
					robot.tapeExtend.setPower(0);
				}

				robot.tapeUpDown2.setPower(y);
				robot.tapeRotate.setPosition(x);
				telemetry.addData("Power UP-DOWN",robot.tapeUpDown2.getPower());
				telemetry.addData("Position L-R",robot.tapeRotate.getPosition());
				telemetry.addData("Position OUT",robot.tapeEncoder.getCurrentPosition());
				telemetry.addData("x",x);
				telemetry.addData("y",y);

			}

			if(test.equals("Elevator")){

				if (gamepad1.dpad_up)
					robot.lifter.setPower(0.75);
				else if (gamepad1.dpad_down)
					robot.lifter.setPower(-.75);
				else
					robot.lifter.setPower(0);


				if (gamepad1.a)
					stored1 = robot.lifter.getCurrentPosition();
				if (gamepad1.b)
					stored2 = robot.lifter.getCurrentPosition();
				if (gamepad1.x)
					stored3 = robot.lifter.getCurrentPosition();
				if (gamepad1.y)
					stored4 = robot.lifter.getCurrentPosition();

				if(gamepad1.dpad_right){
					robot.lightsaber.setPosition(0.45);
				}
				else if (gamepad1.dpad_left){
					robot.lightsaber.setPosition(0);

				}

				if(gamepad1.left_bumper && gamepad1.right_bumper){
					robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				}

				telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
				telemetry.addData("Lift Power", robot.lifter.getPower());
				telemetry.addData("Color Distance", robot.blocksensor_distance.getDistance(DistanceUnit.INCH));
				telemetry.addData("Stored Value 1", stored1);
				telemetry.addData("Stored Value 2", stored2);
				telemetry.addData("Stored Value 3", stored3);
				telemetry.addData("Stored Value 4", stored4);
				telemetry.update();
			}

			if(test.equals("Duck Extend / Spin")){
				if (gamepad1.a) {
					robot.duckextend.setPower(0.85);
				}
				else if (gamepad1.b) {
					robot.duckextend.setPower(-.85);
				}
				else {
					robot.duckextend.setPower(0);
				}
				if (gamepad1.x) {
					spin();
				}

				if (gamepad1.right_trigger != 0) {
					robot.duckspin.setPower(gamepad1.right_trigger);
				}
				else if (gamepad1.left_trigger != 0){
					robot.duckspin.setPower(-1*gamepad1.left_trigger);
				}
				else {
					robot.duckspin.setPower(0);
				}

				if(gamepad1.left_bumper && gamepad1.right_bumper){
					robot.duckextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					robot.duckextend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				}

				telemetry.addData("Ducky CP", robot.duckextend.getCurrentPosition());
				telemetry.addData("Ducky Power (RED CONFIG)", robot.duckspin.getPower());
				telemetry.update();
			}

			if(test.equals("Distances + Color")){




				if(gamepad1.a){

				}
				if(gamepad1.b){

					BallReadingA.add(robot.blocksensor.alpha());
					BallReadingB.add(robot.blocksensor.blue());
					BallReadingR.add(robot.blocksensor.red());
					BallReadingG.add(robot.blocksensor.green());
					BallReadingARGB.add(robot.blocksensor.argb());

				}
				if(gamepad1.x){

				}
				if(gamepad1.y){
					blockReadingA.add(robot.blocksensor.alpha());
					blockReadingB.add(robot.blocksensor.blue());
					blockReadingR.add(robot.blocksensor.red());
					blockReadingG.add(robot.blocksensor.green());
					blockReadingARGB.add(robot.blocksensor.argb());
				}

				if(gamepad1.left_bumper && gamepad1.right_bumper){
					max = Integer.MIN_VALUE;
					min = Integer.MAX_VALUE;
					blockReadingA.clear();
					blockReadingB.clear();
					blockReadingR.clear();
					blockReadingG.clear();
					blockReadingARGB.clear();
				}
				//telemetry.addData("Elevator Reading", robot.elevatorSensor.getDistance(DistanceUnit.INCH));
				telemetry.update();

			}

			if(test.equals("Wheels")){
				if (gamepad1.a) {
					robot.rearLeftMotor.setPower(0.5);
				}
				if (gamepad1.b) {
					robot.rearRightMotor.setPower(0.5);
				}
				if (gamepad1.x) {
					robot.frontLeftMotor.setPower(0.5);
				}
				if (gamepad1.y) {
					robot.frontRightMotor.setPower(0.5);
				}

				telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
				telemetry.addData("Lift Power", robot.lifter.getPower());
				telemetry.update();
			}

			if(test.equals("Elevator Auto")){
				robot.lightsaber.setPosition(0.4);

				if(gamepad1.a){
					robot.raiseToLayer(0);
				}
				if(gamepad1.b){
					robot.raiseToLayer(1);
				}
				if(gamepad1.x){
					robot.raiseToLayer(2);
				}
				if(gamepad1.y){
					robot.raiseToLayer(3);
				}

				telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
				telemetry.addData("Lift Power", robot.lifter.getPower());
				telemetry.update();

			}
			telemetry.update();

			if(isStopRequested()){
				break;
			}

		}

	}

	private void spin() {
		ElapsedTime spinTime = new ElapsedTime();
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
		tfodParameters.minResultConfidence = 0.65f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
		FtcDashboard.getInstance().startCameraStream(tfod,0);
	}
}
