package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;


@TeleOp(name = "GearsTeleOp", group = "Linear Opmode")
public class Tele_op_ritw extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_20_21 robot = new Hardware_20_21(); // use the class created to define a robot's hardware

    // Minimum rotational position

    private ElapsedTime runtime = new ElapsedTime();






   /*
    Code to run ONCE when the driver hits INIT
   */

    public void runOpMode() {

        waitForStart();
        runtime.reset();
        robot.init(hardwareMap, this);


        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        while (opModeIsActive()) {
            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Hello Iron Gears");//
            telemetry.update();



//------------------------------------------------------------------------------------


            if (gamepad2.dpad_up) {
                robot.intakemotor.setPower(0.5);
                robot.intakeservo.setPower(-0.5);
            } else if (gamepad2.dpad_down) {
                robot.intakemotor.setPower(-0.5);
                robot.intakeservo.setPower(0.5);
            } else {
                robot.intakemotor.setPower(0);
                robot.intakeservo.setPower(0);
            }


//------------------------------------------------------------------------------------


            if (gamepad2.y) {

                robot.launcher1.setPower(0.75);

            } else if (gamepad2.b) {

                robot.launcher1.setPower(0);
            }



            if (gamepad2.right_stick_button) {
                robot.launcher1.setPower(0.8);
            } else if (gamepad2.left_stick_button) {
                robot.launcher1.setPower(0.6);
            }




//------------------------------------------------------------------------------------


            if (gamepad1.dpad_right) {
                robot.wobblehand.setPosition(0.35);


            } else if (gamepad1.dpad_left) {
                robot.wobblehand.setPosition(1);

            }


            if (gamepad1.dpad_down) {
                robot.wobble.setPosition(0.25);


            } else if (gamepad1.dpad_up) {
                robot.wobble.setPosition(1);

            }
//--------------------------------------------------------------------------------------

            if (gamepad1.dpad_right) {
                robot.wobblehand2.setPosition(0.35);


            } else if (gamepad1.dpad_left) {
                robot.wobblehand2.setPosition(1);

            }


            if (gamepad1.dpad_down) {
                robot.wobble2.setPosition(0.15);


            } else if (gamepad1.dpad_up) {
                robot.wobble2.setPosition(1);

            }


//------------------------------------------------------------------------------------


            if (gamepad2.x) {

                robot.rearRightMotor.setPower(0);
                robot.rearLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.frontLeftMotor.setPower(0);

                robot.forks.setPosition(0.22);
                sleep(250);

                robot.kicker.setPosition(0.6);
                sleep(250);

                robot.kicker.setPosition(1);
                sleep(50);

                robot.conveyor.setPower(1);
                sleep(750);

                robot.forks.setPosition(0.26);
                sleep(350);

                robot.kicker.setPosition(0.6);
                sleep(50);

                robot.conveyor.setPower(1);
                sleep(1250);

            } else {
                robot.forks.setPosition(0.05);

                robot.kicker.setPosition(1);

                robot.conveyor.setPower(0);

            }

            // One button for forks and kicker

            if (gamepad2.a) {

                robot.rearRightMotor.setPower(0);
                robot.rearLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.frontLeftMotor.setPower(0);

                robot.forks.setPosition(0.27);
                sleep(250);

                robot.kicker.setPosition(0.6);
                sleep(250);

                robot.conveyor.setPower(1);
                sleep(700);

            } else {
                robot.kicker.setPosition(1);

                robot.forks.setPosition(0.05);

                robot.conveyor.setPower(0);

            }


//------------------------------------------------------------------------------------


            if (gamepad2.right_bumper) {
                robot.conveyor.setPower(0.95);
            } else if (gamepad2.left_bumper) {
                robot.conveyor.setPower(-0.95);
            } else {
                robot.conveyor.setPower(0);
            }


//------------------------------------------------------------------------------------


          /*
            if (gamepad2.dpad_right) {
                robot.forks.setPosition(0.27);

            } else {
                robot.forks.setPosition(0.05);

            }

           */


//------------------------------------------------------------------------------------

            if (robot.digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                robot.robotsleep(0);
                sleep(10000);
            }

//------------------------------------------------------------------------------------


            double powermotor = 1;

            if (gamepad1.right_bumper) {
                powermotor = 0.75;
            } else if (gamepad1.left_bumper) {
                powermotor = 0.35;
            }


            double frontleft = (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
            double rearleft = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
            double rearright = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;
            double frontright = (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;


            robot.frontLeftMotor.setPower(frontleft);
            robot.rearLeftMotor.setPower(rearleft);
            robot.rearRightMotor.setPower(rearright);
            robot.frontRightMotor.setPower(frontright);


//------------------------------------------------------------------------------------


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
            telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
            telemetry.addData("launcher1", " %.0f",robot.launcher1.getVelocity()/28*60);
            telemetry.addData("launcher2", "%.0f",robot.launcher2.getVelocity()/28*60);
            telemetry.addData("DSRearLeft", String.format("%.01f in", robot.DSRearLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftBack", String.format("%.01f in", robot.DSLeftBack.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftFront", String.format("%.01f in", robot.DSLeftFront.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSRearRight", String.format("%.01f in", robot.DSRearRight.getDistance(DistanceUnit.INCH)));
            telemetry.update();


//------------------------------------------------------------------------------------

        }


    }
}
