package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;


@TeleOp(name = "DEBUGGER", group = "ZDEBUG")
public class DEBUG extends LinearOpMode {

	Hardware_21_22 robot = new Hardware_21_22();

	private ElapsedTime runtime = new ElapsedTime();

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
				test = "Element Arm";
			}
			else if (gamepad1.dpad_down) {
				test = "Intake Algorithm";
			}

			telemetry.addData("Testing", test);
			telemetry.addLine("Press buttons to change what is tested\n" +
					"\t"+ ins + "\tA - Duck Extender and Spinner\n" +
					"\t"+ ins + "\tB - Elevator\n" +
					"\t"+ ins + "\tY - Distances and Color Sensors\n" +
					"\t"+ ins + "\tX - Wheels\n" +
					"\t"+ ins + "\tDPAD_UP - Tape Measure");
			telemetry.update();

			if(isStopRequested()){
				break;
			}

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


		while (opModeIsActive()) {
			robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
			robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);

			if(test.equals("Elevator")){

				if (gamepad1.dpad_up)
					robot.lifter.setPower(0.5);
				else if (gamepad1.dpad_down)
					robot.lifter.setPower(-.5);
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
					robot.lightsaber.setPosition(0.5);
				}
				else if (gamepad1.dpad_left){
					robot.lightsaber.setPosition(1);

				}

				if(gamepad1.left_bumper && gamepad1.right_bumper){
					robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				}

				telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
				telemetry.addData("Lift Power", robot.lifter.getPower());
				telemetry.addData("Color Distance", robot.blocksensor_distance.getDistance(DistanceUnit.INCH));
				telemetry.addData("Box Distance", robot.distance3.getDistance(DistanceUnit.INCH));
				telemetry.addData("Stored Value 1", stored1);
				telemetry.addData("Stored Value 2", stored2);
				telemetry.addData("Stored Value 3", stored3);
				telemetry.addData("Stored Value 4", stored4);
				telemetry.update();
			}

			if(test.equals("Duck Extend / Spin")){
				if (gamepad1.a) {
					robot.duckextend.setPower(0.5);
				}
				else if (gamepad1.b) {
					robot.duckextend.setPower(-.5);
				}
				else {
					robot.duckextend.setPower(0);
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


				if (gamepad1.dpad_up){
					robot.distancearmservo1.setPosition(.3);
					robot.distancearmservo2.setPosition(.31);
				}
				if (gamepad1.dpad_down){
					robot.distancearmservo1.setPosition(.98);
					robot.distancearmservo2.setPosition(1);
				}

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

				telemetry.addData("Reading 1", robot.distance1.getDistance(DistanceUnit.INCH));
				telemetry.addData("Reading 2", robot.distance2.getDistance(DistanceUnit.INCH));
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

			if(test.equals("Element Arm")){

				/*
				if (gamepad1.dpad_up){
					robot.elementarm.setPosition(0);
				}
				if (gamepad1.dpad_down){
					robot.elementarm.setPosition(0.3);
				}
				if (gamepad1.dpad_left){
					robot.elementarm.setPosition(0.4);
				}
				if (gamepad1.dpad_right){
					robot.elementarm.setPosition(0.35);
				}
				if (gamepad1.a){
					robot.elementclamp2.setPosition(robot.open);
					robot.elementclamp1.setPosition(robot.open);
				}
				if (gamepad1.b){
					robot.elementclamp2.setPosition(robot.closed);
					robot.elementclamp1.setPosition(robot.closed);

				}

				 */

				if (gamepad1.dpad_right){
					robot.TapeMeasureThing.setPower(0.8); // this one go out
				}
				else if (gamepad1.dpad_left){
					robot.TapeMeasureThing.setPower(-0.8);
				}
				else{
					robot.TapeMeasureThing.setPower(0);
				}


				//telemetry.addData("TARGET", robot.elementarm.getPosition());
				//telemetry.addData("Elevator Reading", robot.elevatorSensor.getDistance(DistanceUnit.INCH));
				telemetry.update();

			}
			telemetry.update();

			if(isStopRequested()){
				break;
			}
		}
	}
}
