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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.vuforia.Frame;
import com.vuforia.Image;

//--------------------------------------------------------------------------------------------------

@TeleOp(name = "Peyton Go Crazy", group = "Concept")
public class localizeRecognition extends LinearOpMode {

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

    Pose2d recognizedHere = new Pose2d();

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap, this);
        drive.setPoseEstimate(new Pose2d(36,-(72-8.25),0));

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 16.0 / 9.0);
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
                            telemetry.addData("Width", recognition.getImageWidth());
                            telemetry.addData("Height", recognition.getImageHeight());

                            i++;

                            if (recognition.getLabel().equals("Ball")){
                                telemetry.addData("Object Detected", "Ball");

                                double min = 0;
                                if(recognition.getHeight() <= recognition.getWidth()){
                                    min = recognition.getHeight();
                                }
                                else{
                                    min = recognition.getWidth();
                                }

                                double displacement = getDisplacement(min);
                                double LR = getLeftRight(displacement,getMidpoint(recognition));
                                displacement = normalizePitch(18.43495,displacement);
                                telemetry.addData("Inches Away", displacement);
                                telemetry.addData("Left/Right from Camera (inches)", getLeftRight(displacement,getMidpoint(recognition)));
                                double roboX = drive.getPoseEstimate().getX();
                                double roboY = drive.getPoseEstimate().getY();
                                double roboHead = drive.getPoseEstimate().getHeading();
                                recognizedHere = new Pose2d(
                                        roboX + (displacement*Math.sin(roboHead) + LR*Math.sin(roboHead+(Math.PI/2))),
                                        roboY + (displacement*Math.cos(roboHead) + LR*Math.cos(roboHead+(Math.PI/2))));

                                telemetry.addData("Position on Field (X,Y)", (recognizedHere.getX() - (recognizedHere.getX()%0.001)) + "," + (recognizedHere.getY() - (recognizedHere.getY()%0.001)));

                            }
                        }
                        telemetry.update();
                    }
                }


                drive.update();
                drive(true);


            }
        }
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

    private double getDisplacement(double minimum){
        double slope = -0.13195;
        double inchesAway = slope*minimum + 38;

        return inchesAway - (inchesAway%0.001);
    }

    private double getMidpoint(Recognition recognition){
        return recognition.getLeft() + (recognition.getWidth()/2);
    }

    private double getLeftRight(double displacement, double midpoint){

        double maxDisplacement = displacement*Math.tan(Math.toRadians(14.4));
        double pixelsMaybe = 640.0/(2*maxDisplacement);
        double rawResult = (midpoint/pixelsMaybe) - maxDisplacement;
        return rawResult - (rawResult%0.001);
    }

    private double normalizePitch(double pitchDownFromZeroDEG, double displacementIn){
        return displacementIn * Math.cos(Math.toRadians(pitchDownFromZeroDEG));
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
    }
}
