/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.code;

import android.graphics.Bitmap;

import androidx.annotation.RequiresPermission;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Frame;
import com.vuforia.Image;

//--------------------------------------------------------------------------------------------------

@TeleOp(name = "Peyton Go Wild", group = "Concept")
public class scanningmoment extends LinearOpMode {

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = { "Ball", "Cube", "Duck", "Marker" };
	private static final String VUFORIA_KEY = "AU2ne7j/////AAABmVnObR/UmkTwlIWslw0M3PhbCKZz1zqnqAPh50b1cKYgW7S2e0sM2P06SmDa+ClCAUh/TLJic+MN9jlOEQi+yW7ytjhnEtMyCBxMktuYhoog8VM7HpWYejdoyWu+KDbPd7820Tt16jfZGxCdiBTdvueekVz1zL2U3oPWSBDM4vdtlXE+l+wreA+SCpqeKvk7TvAgo7mk2HcqV6TZ5oB6HeTlYUhjds+x2mZ/7G0hLiEgXZlpcpP8uPAow5H1wci/0H6yx1sTylMPUGBiGQhpOBaKEmVwWZLwk/Zggfissqu3qUGXH84menZWlPv5IMDWSiBmLtoTxx4VVv/env9+v2LS0C8LiD/P+c3msMiLTM1E";
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;

	Hardware_21_22 robot = new Hardware_21_22();
	double frontleft;
	double frontright;
	double rearleft;
	double rearright;

	Pose2d recognizedHere1 = new Pose2d();
	Pose2d recognizedHere2 = new Pose2d();
	Pose2d recognizedHere3 = new Pose2d();
	Pose2d recognizedHere4 = new Pose2d();
	Pose2d recognizedHere5 = new Pose2d();
	Pose2d recognizedHere6 = new Pose2d();
	Pose2d lastRobotRead   = new Pose2d();

	ArrayList<Pose2d> Readings = new ArrayList<>();

	ElapsedTime readTime = new ElapsedTime();

	@Override
	public void runOpMode() {
		initVuforia();
		initTfod();


		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		robot.init(hardwareMap, this);
		//drive.setPoseEstimate(new Pose2d(36,-(72-8.25),0));
		drive.setPoseEstimate(new Pose2d(0, 0,0));

		if (tfod != null) {
			tfod.activate();
			tfod.setZoom(1, 4.0/3.0);
		}

		/** Waits for the game to begin */
		telemetry.addData(">", "Press Play to start op mode");
		telemetry.update();
		waitForStart();

		if (opModeIsActive()) {
			while (opModeIsActive()) {

				//tensor flow stuff here
				if (tfod != null) {

					List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

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

							telemetry.addData("width", recognition.getWidth());
							telemetry.addData("Distance", getDisplacement(recognition));

							i++;

							if (recognition.getLabel().equals("Ball")){
								Pose2d here = big(recognition,drive);
								valid(Readings,here);
							}
						}

						int count = 0;
						for(Pose2d pose : Readings){
							telemetry.addLine("Stored Pose :" + count);
							telemetry.addData("X", pose.getX() - pose.getX()%0.05);
							telemetry.addData("Y", pose.getY() - pose.getY()%0.05);
							count++;
						}
						telemetry.update();
					}
				}


				drive.update();
				drive(true);


			}
		}
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
		Readings.add(here);
		return true;
	}


	public void drive(boolean D) {

		double powermotor = .52;

		if (gamepad1.right_bumper) {
			powermotor = 1;
		} else if (gamepad1.left_bumper) {
			powermotor = .35;
		}

		if (D) {
			rearleft = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
			frontleft = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
			frontright = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
			rearright = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
		}
		else {
			frontleft = (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
			rearleft = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
			rearright = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;
			frontright = (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;
		}

		robot.frontLeftMotor.setPower(frontleft);
		robot.rearLeftMotor.setPower(rearleft);
		robot.rearRightMotor.setPower(rearright);
		robot.frontRightMotor.setPower(frontright);

	}

	private Pose2d big(Recognition recognition, SampleMecanumDrive drive){

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

		//FtcDashboard.getInstance().startCameraStream(vuforia,0);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.7f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

		FtcDashboard.getInstance().startCameraStream(tfod,0);
	}
}
