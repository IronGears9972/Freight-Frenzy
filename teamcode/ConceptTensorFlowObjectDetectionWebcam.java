package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Autonomous(name = "Scanning and Run", group = "Concept")
public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    Hardware_20_21 robot = new Hardware_20_21();
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private static final String VUFORIA_KEY =
            "Abvr6pj/////AAABmXhem3EDYkB2l9ldqIzfhPY1eAddpO0PM4c17wFb6y3q8p1VMAJM35J7Bt+HKLFUV0ibDyiAYunShwlSyug/FKJpY+GG7pYUaEx44wkOWFkTSGCoE8lcZO4J55tv9rLsqIU0adRZDCkHrUMeV7zQecbuAqnQ+3/iBPdzgNm8trMkaCJ/k/ulJw5uAraNQC25kaom1XGWx6vlftXyTGszAfvFHVvShrn3JBuf5ZYEkIAZ6wtiYoxss3gROnkdGbI7kW2pD1jZdey39WNrHDXSS0Wt4SLl0O0BgFskK+eXAIKxy8f8fgPXauk8fNT4+ZUTus+gBt8xZ6Xs/A8O5CpC4l8sp0w4c/gZW7DpNSsU+Qrk";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        List<Recognition> updatedRecognitions = tfod.getRecognitions();
        String readValue = "none";

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.

                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        readValue = UpdateDisplay(updatedRecognitions, readValue);
                        RunFromReadValue(readValue);
                        break;

                    }
                }
            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private String UpdateDisplay(List<Recognition> updatedRecognitions, String readValue) {
        telemetry.addData("# Object Detected", updatedRecognitions.size());
        // step through the list of recognitions and display boundary info.
        int i = 0;
        updatedRecognitions.get(0).getLabel();
        for (Recognition recognition : updatedRecognitions) {
            readValue = recognition.getLabel();
            recognition.getConfidence();
            telemetry.addData(String.format("label (%d)", i), readValue);
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
        }
        telemetry.update();
        return readValue;

    }

    private void RunFromReadValue(String readValue) {
        if (readValue.equals(LABEL_SECOND_ELEMENT)) {
            telemetry.addData("Say", "Auto One");//
            AutoOne();
        } else if (readValue.equals(LABEL_FIRST_ELEMENT)) {
            telemetry.addData("Say", "Auto Four");//
            AutoFour();
        } else {
            telemetry.addData("Say", "Auto Zero");//
            AutoZero();
        }
    }

    private void AutoOne() {

        float hsvValues[] = {0F, 0F, 0F};


        final float values[] = hsvValues;



        robot.init(hardwareMap, this);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
            telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
            telemetry.addData("launcher1", " %.0f", robot.launcher1.getVelocity() / 28 * 60);
            telemetry.addData("DSRearLeft", String.format("%.01f in", robot.DSRearLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftBack", String.format("%.01f in", robot.DSLeftBack.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftFront", String.format("%.01f in", robot.DSLeftFront.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSRearRight", String.format("%.01f in", robot.DSRearRight.getDistance(DistanceUnit.INCH)));
            Orientation angels = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("imu , first/second/third", "%.1f %.1f %.1f", angels.firstAngle, angels.secondAngle, angels.thirdAngle);
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();


        }


        robot.wobblehand2.setPosition(0.35);
        robot.drivestraight(-27, 0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(-10,0.3);

        robot.launcher1.setPower(0.60);
        sleep(1100);

        robot.launcher1.setPower(0.60);



        //robot.conveyor.setTargetPosition(4800);//300
        robot.conveyor.setPower(-0.95);
        //robot.kicker.setPosition(0.6);
        sleep(175);

        robot.launcher1.setPower(0);//
        robot.conveyor.setPower(0);
        sleep(50);//

        robot.forks.setPosition(0.24);
        sleep(375);

        robot.conveyor.setPower(-0.35);//-0.95
        //robot.kicker.setPosition(1);
        sleep(250);//400//200



        robot.conveyor.setPower(0);//-0.95
        robot.forks.setPosition(0.05);//0.29
        //sleep(300);

        robot.drivestrafe(3,0.3);//

        //robot.kicker.setPosition(0.6);
        //sleep(50);

        //robot.conveyor.setPower(-0.95);
        //sleep(950);


        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);

        //robot.forks.setPosition(0.05);
        //robot.kicker.setPosition(1);




        robot.intakemotor.setPower(-0.95);
        robot.intakeservo.setPower(-0.8);

        robot.drivestraight(-12, 0.3);
        sleep(500);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestraight(-26, 0.3);
        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(0.8);
        sleep(700);

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);

        /*

        robot.launcher1.setPower(0.95);
        sleep(1100);

        robot.launcher1.setPower(0.95);




        robot.conveyor.setPower(-0.95);

        robot.forks.setPosition(0.30);
        sleep(500);

        robot.kicker.setPosition(0.6);
        sleep(175);






        robot.robotsleep(0);
        sleep(1000);

        robot.conveyor.setPower(0);
        robot.launcher1.setPower(0);


         */

        robot.drivestrafe(-4,0.3);//


        robot.launcher1.setPower(0.90);
        sleep(1350);


        robot.launcher1.setPower(0.90);


        robot.conveyor.setPower(-0.95);
        //robot.forks.setPosition(0.24);
        sleep(275);


        //robot.kicker.setPosition(0.6);
        //sleep(175);

        robot.conveyor.setPower(0.35);
        robot.forks.setPosition(0.24);
        //robot.kicker.setPosition(1);
        sleep(400);

        robot.conveyor.setPower(-0.95);
        sleep(350);
        robot.forks.setPosition(0.29);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(-0.95);
        sleep(950);


        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);


        robot.drivestraight(-50, 0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(-6,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.wobble2.setPosition(0.15);
        sleep(700);

        robot.wobblehand2.setPosition(0.85);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(-35,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.wobblekeep.setPosition(1);

        robot.drivestraight(111.5,0.6);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe2(36,0.10);

        robot.wobblehand2.setPosition(0.35);
        sleep(750);

        robot.drivestraight(-106, 0.6);//0.5

        robot.wobblekeep.setPosition(0.4);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(13, 0.4);

        robot.wobblehand2.setPosition(1);
        sleep(400);

        robot.wobble2.setPosition(0.25);

        robot.drivestraight(24, 0.7);

        robot.robotsleep(0);
        sleep(200);


    }
    //              ^ 1 Ring

    private void AutoZero() {

        robot.init(hardwareMap, this);

        OpRunENCODERZero();

        waitForStart();

        robot.wobblehand2.setPosition(0.35);
        robot.drivestraight(-65,0.3);

        robot.drivestrafe(-10,-0.3);

        OpRunShooterZero();

        robot.drivestraight(-22,0.3);

        new ConceptTensorFlowObjectDetectionWebcam.OpRunDropWobble2AndPutDoneWobble1Zero().invoke();

        robot.drivestrafe(-60,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.wobblekeep.setPosition(1);

        robot.drivestraight(84,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe2(20,0.12);

        robot.wobblehand2.setPosition(0.35);
        sleep(650);

        robot.wobble2.setPosition(0.20);

        robot.drivestraight(-90,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(48,0.35);

        robot.robotsleep(0);
        sleep(200);

        robot.wobblehand2.setPosition(1);
        robot.wobblekeep.setPosition(0.4);
        sleep(1000);

        robot.wobble2.setPosition(1);

        robot.drivestrafe(-20,0.6);

        robot.drivestraight(12,0.5);


        OpRunTelemetryZero();


    }
    //              ^ 0 Ring

    private void AutoFour() {
        robot.init(hardwareMap, this);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (!opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
            telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
            telemetry.addData("launcher1", " %.0f", robot.launcher1.getVelocity() / 28 * 60);
            telemetry.addData("DSRearLeft", String.format("%.01f in", robot.DSRearLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftBack", String.format("%.01f in", robot.DSLeftBack.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftFront", String.format("%.01f in", robot.DSLeftFront.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSRearRight", String.format("%.01f in", robot.DSRearRight.getDistance(DistanceUnit.INCH)));
            Orientation angels = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("imu , first/second/third", "%.1f %.1f %.1f", angels.firstAngle, angels.secondAngle, angels.thirdAngle);
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.update();



        }

        robot.wobblehand2.setPosition(0.35);
        robot.drivestraight(-25, 0.3);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestrafe(8,0.3);

        robot.robotsleep(0);
        sleep(100);

        robot.launcher1.setPower(0.80);

        robot.drivestraight(-41, 0.3);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestrafe(-18,0.3);

        robot.robotsleep(0);
        sleep(100);

        //robot.launcher1.setPower(0.95);
        //sleep(1400);

        robot.launcher1.setPower(0.80);


        robot.conveyor.setPower(-0.95);
        //robot.forks.setPosition(0.24);
        sleep(275);

        //robot.kicker.setPosition(0.6);
        //sleep(175);

        robot.conveyor.setPower(0);
        robot.forks.setPosition(0.24);
        //robot.kicker.setPosition(1);
        sleep(400);

        robot.conveyor.setPower(-0.95);
        sleep(350);
        robot.forks.setPosition(0.29);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(-0.95);
        sleep(500);//750


        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);
        robot.forks.setPosition(0.05);
        robot.kicker.setPosition(1);

        robot.drivestraight(-67, 0.5);

        robot.robotsleep(0);
        sleep(200);

        robot.wobble2.setPosition(0.15);
        sleep(600);

        robot.drivestrafe(20,0.3);

        robot.wobblehand2.setPosition(0.85);
        sleep(400);

        robot.drivestrafe(-48,0.4);

        robot.robotsleep(0);
        sleep(200);

        robot.wobblekeep.setPosition(1);

        robot.drivestraight(135, 0.6);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe2(36,0.10);

        robot.wobblehand2.setPosition(0.34);
        sleep(750);

        robot.drivestrafe(28,0.3);

        robot.intakemotor.setPower(-0.95);
        robot.intakeservo.setPower(-0.8);

        robot.launcher1.setPower(0.55);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestraightleft(-36, 0.3);//50

        robot.robotsleep(0);//
        sleep(1000);//600

        robot.drivestraight(-6, 0.3);

        robot.robotsleep(0);//
        sleep(1200);//600

        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(0.8);
        sleep(500);//700

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);


        //robot.launcher1.setPower(0.75);
        //sleep(1350);


        robot.launcher1.setPower(0.55);


        robot.conveyor.setPower(-0.95);
        //robot.forks.setPosition(0.24);
        sleep(275);


        //robot.kicker.setPosition(0.6);
        //sleep(175);

        robot.conveyor.setPower(0.35);
        robot.forks.setPosition(0.24);
        //robot.kicker.setPosition(1);
        sleep(400);

        robot.conveyor.setPower(-0.95);
        sleep(350);
        robot.forks.setPosition(0.29);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(-0.95);
        sleep(950);

        robot.conveyor.setPower(0);
        robot.forks.setPosition(0.05);
        robot.kicker.setPosition(1);


        robot.intakemotor.setPower(-0.95);
        robot.intakeservo.setPower(-0.8);

        robot.launcher1.setPower(0.75);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestraightleft(-36, 0.3);//50

        robot.robotsleep(0);//
        sleep(1000);//600

        robot.drivestraight(-6, 0.3);

        robot.robotsleep(0);//
        sleep(1200);//600

        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(0.8);
        sleep(500);//700

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);



    }
    //              ^ 4 Rings


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        //TODO: Maybe take this out
        tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    private void OpRunENCODERZero() {
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //              ^ Starts all Encoders

    private void OpRunShooterZero() {
        robot.launcher1.setPower(0.90);
        sleep(1500);

        robot.launcher1.setPower(0.90);


        robot.conveyor.setPower(-0.95);
        //robot.forks.setPosition(0.24);
        sleep(275);

        //robot.kicker.setPosition(0.6);
        //sleep(175);

        robot.conveyor.setPower(0);
        robot.forks.setPosition(0.24);
        //robot.kicker.setPosition(1);
        sleep(400);

        robot.conveyor.setPower(-0.95);
        sleep(350);
        robot.forks.setPosition(0.29);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(-1);
        sleep(950);


        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);
    }
    //              ^ Runs the Shooter and Forks

    private void OpRunTelemetryZero() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
        telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
        telemetry.addData("launcher1", " %.0f",robot.launcher1.getVelocity()/28*60);
        telemetry.update();
    }
    //              ^ Outputs text so we can read data

    public class OpRunDropWobble2AndPutDoneWobble1Zero {
        public void invoke() {
            robot.robotsleep(0);
            sleep(100);

            robot.wobble2.setPosition(0.15);
            sleep(800);

            robot.drivestrafe(23,-0.3);

            robot.wobblehand2.setPosition(1);
            sleep(500);

            robot.robotsleep(0);
            sleep(200);
        }
    }
    //              ^ Grabs 2nd wobble goal and drops in square

}
