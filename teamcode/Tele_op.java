package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "GearsTeleOp", group = "Linear Opmode")
public class Tele_op extends LinearOpMode {

    Hardware_21_22 robot = new Hardware_21_22();

    private ElapsedTime runtime = new ElapsedTime();

    int tele = 3;
    int TargetL = 0;
    double frontleft = 0;
    double rearleft = 0;
    double rearright = 0;
    double frontright = 0;

    boolean driver = false;

    //private ShootPSSM shootPSSM=new ShootPSSM();


    public void runOpMode() {

        telemetry.addData("Say", "Hello Iron Gears");
        telemetry.update();

        while (!(opModeIsActive())){
            //
            if(gamepad1.right_stick_button) {
                driver = true;
            }
            else if (gamepad1.left_stick_button) {
                driver = false;
            }
            telemetry.addData("Controls", driver);
            telemetry.update();
        }

        waitForStart();
        runtime.reset();
        robot.init(hardwareMap, this);

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.lifter.setTargetPosition(0);
        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.duckextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.elementclamp.setPosition(0);
        robot.elementarm.setPosition(0.55);
        robot.lightsaber.setPosition(0);


        while (opModeIsActive()) {
//------------------------------------------------------------------------------------

            if (robot.lifter.getCurrentPosition() < 50) {
                if (gamepad2.right_trigger == 1) {
                    robot.intakemotor.setPower(-.8);
                } else if (gamepad2.left_trigger == 1) {
                    robot.intakemotor.setPower(.8);
                } else {
                    robot.intakemotor.setPower(0);
                }
            } else {
                robot.intakemotor.setPower(0);
            }


//------------------------------------------------------------------------------------

            if (gamepad2.y && !gamepad2.left_bumper) {
                while(gamepad2.y) {
                    robot.duckspin.setPower(0.8);
                    robot.duckspinblue.setPower(0.8);
                }
                robot.duckspin.setPower(-.2);
                robot.duckspinblue.setPower(-.2);
            }
            else if (gamepad2.y && gamepad2.left_bumper) {
                robot.duckspin.setPower(-.2);
                robot.duckspinblue.setPower(-.2);
            }
            else {
                robot.duckspin.setPower(0);
                robot.duckspinblue.setPower(0);
            }

//------------------------------------------------------------------------------------

            if (gamepad2.x) {
                robot.lightsaber.setPosition(0.5);
            } else {
                robot.lightsaber.setPosition(0.05);
            }

//------------------------------------------------------------------------------------

            if (gamepad2.dpad_left) {
                robot.duckextend.setPower(.9);
            } else if (gamepad2.dpad_right) {
                robot.duckextend.setPower(-.9);
            } else {
                robot.duckextend.setPower(0);

//------------------------------------------------------------------------------------

                if (robot.blocksensor.argb() > 1400000000 && robot.blocksensor.argb() < 1800000000) {
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                }
                else if (robot.blocksensor.argb() > 410000000 && robot.blocksensor.argb() < 490000000) {
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                }
                else if (robot.blocksensor.argb() < -1500000000 && robot.blocksensor.argb() > -2100000000) {
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                }
                else if(robot.blocksensor.argb() < 500000000 && robot.blocksensor.argb() > 600000000) {
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                }
                else {
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                }


//------------------------------------------------------------------------------------

                if (!gamepad2.left_bumper && gamepad2.a) {
                    robot.lifter.setTargetPosition(robot.bottom);
                }
                else if (!gamepad2.left_bumper && gamepad2.b) {
                    robot.lifter.setTargetPosition(robot.layer2);
                }
                else if (gamepad2.left_bumper && gamepad2.a) {
                    robot.lifter.setTargetPosition(robot.layer3);
                }
                else if (gamepad2.left_bumper && gamepad2.b) {
                    robot.lifter.setTargetPosition(robot.TipTop);
                }
                else if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    robot.lifter.setTargetPosition(-10);
                }

                if (gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1){
                    robot.lifter.setTargetPosition(robot.lifter.getTargetPosition()-10);
                }

                if (robot.lifter.getCurrentPosition() <= (robot.lifter.getTargetPosition()-30) || robot.lifter.getCurrentPosition() >= (robot.lifter.getTargetPosition()+30)){
                    robot.lifter.setPower(0.9);
                }
                else {
                    robot.lifter.setPower(0);
                }
//------------------------------------------------------------------------------------

                double powermotor = 0.3;

                if (gamepad1.right_bumper) {
                    powermotor = .95;
                } else if (gamepad1.left_bumper) {
                    powermotor = 0.2;
                }

//------------------------------------------------------------------------------------

                if(driver = true) {
                    frontleft = (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
                    rearleft = (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
                    rearright = (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
                    frontright = (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;

                    robot.frontLeftMotor.setPower(frontleft);
                    robot.rearLeftMotor.setPower(rearleft);
                    robot.rearRightMotor.setPower(rearright);
                    robot.frontRightMotor.setPower(frontright);

                    if (gamepad2.left_bumper && gamepad2.dpad_up) {
                        robot.elementclamp.setPosition(.5);
                    } else if (gamepad2.left_bumper && gamepad2.dpad_down) {
                        robot.elementclamp.setPosition(0);
                    }

                    if (!gamepad2.left_bumper && gamepad2.dpad_down) {
                        robot.elementarm.setPosition(0);
                    } else if (!gamepad2.left_bumper && gamepad2.dpad_up) {
                        robot.elementarm.setPosition(0.25);
                    } else if (!gamepad2.right_bumper && gamepad2.dpad_up) {
                        robot.elementarm.setPosition(0.5);
                    }
                } else {

                    frontleft = (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
                    rearleft = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor;
                    rearright = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;
                    frontright = (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor;

                    robot.frontLeftMotor.setPower(frontleft);
                    robot.rearLeftMotor.setPower(rearleft);
                    robot.rearRightMotor.setPower(rearright);
                    robot.frontRightMotor.setPower(frontright);

                    if (!gamepad2.left_bumper && gamepad2.dpad_up) {
                        robot.elementclamp.setPosition(.5);
                    } else if (!gamepad2.left_bumper && gamepad2.dpad_down) {
                        robot.elementclamp.setPosition(0);
                    }

                    if (gamepad2.left_bumper && gamepad2.dpad_down) {
                        robot.elementarm.setPosition(0);
                    } else if (gamepad2.left_bumper && gamepad2.dpad_up) {
                        robot.elementarm.setPosition(0.25);
                    } else if (gamepad2.right_bumper && gamepad2.dpad_up) {
                        robot.elementarm.setPosition(0.5);
                    }
                }

//------------------------------------------------------------------------------------

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("SM",shoot3SM.state.toString());
                //telemetry.addData("SM",shoot1SM.statea.toString());
                telemetry.addData("frontRightMotor", robot.frontRightMotor.getCurrentPosition());
                telemetry.addData("rearLeftMotor", robot.rearLeftMotor.getCurrentPosition());
                telemetry.addData("rearRightMotor", robot.rearRightMotor.getCurrentPosition());
                telemetry.addData("frontLeftMotor", robot.frontLeftMotor.getCurrentPosition());
                telemetry.addData("duckExtend", robot.duckextend.getCurrentPosition());
                telemetry.addData("DR", robot.distanceR.getDistance(DistanceUnit.INCH));
                telemetry.addData("DL", robot.distanceL.getDistance(DistanceUnit.INCH));
                telemetry.addData("BS-A", robot.blocksensor.alpha());
                telemetry.addData("BS-R", robot.blocksensor.red());
                telemetry.addData("BS-G", robot.blocksensor.green());
                telemetry.addData("BS-B", robot.blocksensor.blue());
                telemetry.addData("BS-ARGB", robot.blocksensor.argb());
                telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
                telemetry.addData("lifterTP", robot.lifter.getTargetPosition());
                telemetry.addData("Lift Power", robot.lifter.getPower());
                telemetry.update();
            }
        }
    }
}