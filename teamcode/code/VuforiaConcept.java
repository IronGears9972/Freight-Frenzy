package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.R;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Vuforia;

@TeleOp(name = "New Vuforia Concept", group = "ZZConcept")

public class VuforiaConcept extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {

		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
		//parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		parameters.vuforiaLicenseKey = "AUHhas//////AAABmTPJKKjZO0gMvlweR8fyCKY8mfIwad4HjmeDXLQhxw4RqdwTgpWds5puQf+Itchttds91IsKzYktpMl30n0o3SBRaZ2ruS1XzwGTQd9tC6Diu/nfo3Dbh5ZWXomyFkOivw1PjoYg2tgKU1eih2fOQXt+2s3KltzRpxX1iBsNbFwBL3uvM3R/GBBibLr2x/JJ0pCdEVTjiaBthnghV3fxJZQUK7eTiiRsO5IosYFc/RzsKW+2c5LO0xCgVPnQYA7piu2vG95lDwC/Eil+ibqOuS2fV2ePiMSkxGFUW9yD0ex5GaCP63S3x/4WLiFqwd+yJTZepUJjjb9B1sKo/XJkUwPK27fRY3yBR0YQXy1AZA9Q";
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Something Cool");
		parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;


		VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
		vuforia.setFrameQueueCapacity(3);

		VuforiaTrackables wallNav = vuforia.loadTrackablesFromAsset("FreightFrenzy");
		wallNav.get(0).setName("Ball?");
		wallNav.get(1).setName("Block?");
		wallNav.get(2).setName("Duck?");
		wallNav.get(3).setName("Marker?");



		waitForStart();

		while(opModeIsActive()){
			for (VuforiaTrackable guy : wallNav){
				OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) guy.getListener()).getPose();

				if(pose != null){
					VectorF translation = pose.getTranslation();

					telemetry.addData(wallNav.getName() + " Translation", translation);


				}
			}
		}
	}
}
