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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "Path", group = "ARED")
public class AlexPath extends LinearOpMode {

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

        Recognition f = new Recognition() {
            @Override
            public String getLabel() {
                return "EVIL";
            }

            @Override
            public float getConfidence() {
                return 1;
            }

            @Override
            public float getLeft() {
                return 100;
            }

            @Override
            public float getRight() {
                return 300;
            }

            @Override
            public float getTop() {
                return 100;
            }

            @Override
            public float getBottom() {
                return 300;
            }

            @Override
            public float getWidth() {
                return 200;
            }

            @Override
            public float getHeight() {
                return 200;
            }

            @Override
            public int getImageWidth() {
                return 640;
            }

            @Override
            public int getImageHeight() {
                return 480;
            }

            @Override
            public double estimateAngleToObject(AngleUnit angleUnit) {
                return 90;
            }
        };



        while(!opModeIsActive()){
            boolean reading = false;
            robot.lightsaber.setPosition(robot.lightsaber45);

            telemetry.addLine("step 10");
            telemetry.update();

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

                            telemetry.update();


                        }


                    }


                }
                telemetry.update();
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