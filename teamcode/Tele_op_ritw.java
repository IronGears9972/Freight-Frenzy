package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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
        robot.launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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

                robot.launcher.setPower(1);

            } else if (gamepad2.b) {

                robot.launcher.setPower(0);
            }



//------------------------------------------------------------------------------------


            if (gamepad1.dpad_right) {
                robot.wobblehand.setPosition(-0.3);


            } else if (gamepad1.dpad_left) {
                robot.wobblehand.setPosition(0.5);

            }


            if (gamepad1.dpad_down) {
                robot.wobble.setPosition(0.25);


            } else if (gamepad1.dpad_up) {
                robot.wobble.setPosition(1);

            }


//------------------------------------------------------------------------------------


            if (gamepad2.x) {

                robot.rearRightMotor.setPower(0);
                robot.rearLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.frontLeftMotor.setPower(0);

                robot.forks.setPosition(0.25);
                sleep(250);

                robot.kicker.setPosition(0.6);
                sleep(250);

                robot.kicker.setPosition(1);
                sleep(50);

                robot.conveyor.setPower(0.95);
                sleep(750);

                robot.forks.setPosition(.3);
                sleep(350);

                robot.kicker.setPosition(0.6);
                sleep(50);

                robot.conveyor.setPower(0.95);
                sleep(1250);

            } else {
                robot.forks.setPosition(0.05);

                robot.kicker.setPosition(1);

            }

            // One button for forks and kicker

            if (gamepad2.a) {

                robot.rearRightMotor.setPower(0);
                robot.rearLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.frontLeftMotor.setPower(0);

                robot.forks.setPosition(0.3);
                sleep(250);

                robot.kicker.setPosition(0.6);
                sleep(250);

                robot.conveyor.setPower(0.95);
                sleep(650);

            } else {
                robot.kicker.setPosition(1);

                robot.forks.setPosition(0.05);

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

            if (gamepad1.dpad_down) {
                robot.hookF.setPosition(1);
                robot.hookR.setPosition(0.1);
            }
            else if(gamepad1.dpad_up) {
                robot.hookF.setPosition(0);
                robot.hookR.setPosition(1);
            }


                */


//------------------------------------------------------------------------------------



           /*
           if (gamepad2.dpad_right) {
               robot.ssgbottom.setPosition(1);
               sleep(1000);
               robot.ssgtop.setPosition(0);
               //OUT & OPEN
           }

           else if(gamepad2.dpad_left) {
               robot.ssgtop.setPosition(0.85);
               sleep(1000);
               robot.ssgbottom.setPosition(.3);
               //IN & CLOSED
           }
           */


//------------------------------------------------------------------------------------


            double power = 1;

            if (gamepad1.right_bumper) {
                power = 0.75;
            } else if (gamepad1.left_bumper) {
                power = 0.35;
            }


            double frontleft = (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * power;
            double rearleft = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * power;
            double rearright = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * power;
            double frontright = (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * power;


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
            telemetry.addData("launcher", " %.0f",robot.launcher.getVelocity()/28*60);
            telemetry.update();


//------------------------------------------------------------------------------------

        }


    }
}
