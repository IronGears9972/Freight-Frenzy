package org.firstinspires.ftc.teamcode.teamcode.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestingTeleOp", group = "Linear Opmode")
public class TestingTeleOp extends LinearOpMode {

    Hardware_21_22 robot = new Hardware_21_22();

    private ElapsedTime runtime = new ElapsedTime();

    double frontleft = 0;
    double rearleft = 0;
    double rearright = 0;
    double frontright = 0;

    public void runOpMode() {


        telemetry.addData("Say", "Hello Iron Gears");
        telemetry.update();

        waitForStart();
        runtime.reset();
        robot.init(hardwareMap, this);

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        robot.lifter.setTargetPosition(0);
        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.duckextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            double powermotor = 0.4;

            if (gamepad1.right_bumper) {
                powermotor = 1;
            } else if (gamepad1.left_bumper) {
                powermotor = 0.05;
            }

            rearleft = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
            frontleft = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
            frontright = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
            rearright = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;

            robot.frontLeftMotor.setPower(frontleft);
            robot.rearLeftMotor.setPower(rearleft);
            robot.rearRightMotor.setPower(rearright);
            robot.frontRightMotor.setPower(frontright);

            /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.frontLeftMotor.setPower(v1);
            robot.frontRightMotor.setPower(v2);
            robot.rearLeftMotor.setPower(v3);
            robot.rearRightMotor.setPower(v4);*/

            if (gamepad1.y) {
                robot.frontRightMotor.setPower(.4);
                robot.frontLeftMotor.setPower(.4);
                robot.rearRightMotor.setPower(.4);
                robot.rearLeftMotor.setPower(.4);
            }
            else if (gamepad1.a) {
                robot.frontRightMotor.setPower(-.4);
                robot.frontLeftMotor.setPower(-.4);
                robot.rearRightMotor.setPower(-.4);
                robot.rearLeftMotor.setPower(-.4);
            }
            else if (gamepad1.right_stick_button) {
                robot.frontRightMotor.setPower(0);
                robot.frontLeftMotor.setPower(0);
                robot.rearRightMotor.setPower(0);
                robot.rearLeftMotor.setPower(0);
            }

            /*if (gamepad2.right_trigger == 1) {
                robot.intakemotor.setPower(-.95);
            } else if (gamepad2.left_trigger == 1) {
                robot.intakemotor.setPower(.95);
            } else {
                robot.intakemotor.setPower(0);
            }

            if (gamepad2.dpad_up) {
                robot.elementclamp1.setPosition(.5);
                robot.elementclamp2.setPosition(.5);
            } else {
                robot.elementclamp1.setPosition(0);
                robot.elementclamp2.setPosition(0);
            }

            if (gamepad2.y && !gamepad2.left_bumper) {
                while (gamepad2.y) {
                    robot.duckspin.setPower(0.9);
                    robot.duckspinblue.setPower(0.9);
                }
                robot.duckspin.setPower(-.2);
                robot.duckspinblue.setPower(-.2);
            } else {
                robot.duckspin.setPower(0);
                robot.duckspinblue.setPower(0);
            }

            if (gamepad2.dpad_left) {
                robot.duckextend.setPower(.9);
            } else if (gamepad2.dpad_right) {
                robot.duckextend.setPower(-.9);
            } else {
                robot.duckextend.setPower(0);
            }

            if (gamepad2.dpad_down) {
                robot.elementarm.setPosition(.3);
            }
            else {
                robot.elementarm.setPosition(0);
            }

            if (!gamepad2.left_bumper && gamepad2.a) {
                robot.lifter.setTargetPosition(robot.bottom);
            } else if (!gamepad2.left_bumper && gamepad2.b) {
                robot.lifter.setTargetPosition(robot.layer2);
            } else if (gamepad2.left_bumper && gamepad2.a) {
                robot.lifter.setTargetPosition(robot.layer3);
            } else if (gamepad2.left_bumper && gamepad2.b) {
                robot.lifter.setTargetPosition(robot.TipTop);
            } else if (gamepad2.left_bumper && gamepad2.right_bumper) {
                robot.lifter.setTargetPosition(0);
            }

            if (gamepad1.x && !gamepad1.left_bumper) {
                robot.lightsaber.setPosition(0.4);
            } else {
                robot.lightsaber.setPosition(0);
            }

            if(gamepad2.a){
             robot.lifter.setTargetPosition(0);
             robot.lifter.setPower(.5);
            }
            else if(gamepad2.b){
                robot.lifter.setTargetPosition(100);
                robot.lifter.setPower(.5);
            }
            else if(gamepad2.x){
                robot.lifter.setTargetPosition(500);
                robot.lifter.setPower(.5);
            }
            else if(gamepad2.y){
                robot.lifter.setTargetPosition(1000);
                robot.lifter.setPower(.5);
            }
            else {
                robot.lifter.setPower(0);
            }

            if(gamepad2.right_bumper) {
                robot.lifter.setPower(0.5);
            }
            else{
                robot.lifter.setPower(0);
            }*/

            telemetry.addData("leftWheel", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("rightWheel", robot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("middleWheel", robot.frontLeftMotor.getCurrentPosition());

            telemetry.addData("FL", robot.frontLeftMotor.getPower());
            telemetry.addData("FR", robot.frontRightMotor.getPower());
            telemetry.addData("RL", robot.rearLeftMotor.getPower());
            telemetry.addData("RR", robot.rearRightMotor.getPower());

            telemetry.addData("FL", frontleft);
            telemetry.addData("FR", frontright);
            telemetry.addData("RL", rearleft);
            telemetry.addData("RR", rearright);

            telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
            telemetry.addData("lifterTP", robot.lifter.getTargetPosition());
            telemetry.addData("Lift Power", robot.lifter.getPower());

            telemetry.addData("Intake Power", robot.intakemotor.getPower());
            telemetry.update();
        }
    }
}
