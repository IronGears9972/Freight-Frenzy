package org.firstinspires.ftc.teamcode.teamcode.teamcode;

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

    double frontleft = 0;
    double rearleft = 0;
    double rearright = 0;
    double frontright = 0;

    boolean driver = false;
    boolean dillon = false;



    public void runOpMode() {

        telemetry.addData("Say", "Hello Iron Gears");
        telemetry.update();

        while (!(opModeIsActive())) {

            if (gamepad1.right_stick_button) {
                driver = true;
            } else if (gamepad1.left_stick_button) {
                driver = false;
            }

            if (gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1 && gamepad1.a) {
                dillon = true;
            } else if (gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1 && gamepad1.b) {
                dillon = false;
            }

            telemetry.addData("Controls", driver);
            telemetry.addData("Squid", dillon);
            telemetry.update();

            if (isStopRequested()) {
                break;
            }
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

        while (opModeIsActive()) {
//------------------------------------------------------------------------------------

            if(dillon == false) {
                if (robot.lifter.getCurrentPosition() < 50) {
                    if (gamepad2.right_trigger == 1) {
                        robot.intakemotor.setPower(-.95);
                    } else if (gamepad2.left_trigger == 1) {
                        robot.intakemotor.setPower(.95);
                    } else {
                        robot.intakemotor.setPower(0);
                    }
                } else {
                    robot.intakemotor.setPower(0);
                }


//------------------------------------------------------------------------------------

                if (gamepad2.y && !gamepad2.left_bumper) {
                    while (gamepad2.y) {
                        robot.duckspin.setPower(0.8);
                        robot.duckspinblue.setPower(0.8);
                    }
                    robot.duckspin.setPower(-.2);
                    robot.duckspinblue.setPower(-.2);
                } else {
                    robot.duckspin.setPower(0);
                    robot.duckspinblue.setPower(0);
                }

//------------------------------------------------------------------------------------

                if (gamepad2.x && !gamepad2.left_bumper) {
                    robot.lightsaber.setPosition(0.4);
                } else {
                    robot.lightsaber.setPosition(0);
                }

//------------------------------------------------------------------------------------

                if (gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1 && gamepad1.a) {
                    driver = false;
                } else if (gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1 && gamepad1.b) {
                    driver = true;
                }

//------------------------------------------------------------------------------------

                if (gamepad2.dpad_left) {
                    robot.duckextend.setPower(.9);
                } else if (gamepad2.dpad_right) {
                    robot.duckextend.setPower(-.9);
                } else {
                    robot.duckextend.setPower(0);

//------------------------------------------------------------------------------------

                    /*if (runtime.seconds() > 0 && runtime.seconds() <= 80) {
                        if (robot.blocksensor.argb() > 0 && robot.blocksensor.argb() < 16000000 ||
                                robot.blocksensor.argb() > 17000000 && robot.blocksensor.argb() < 33000000 ||
                                robot.blocksensor.argb() > 34000000 && robot.blocksensor.argb() < 50000000 ||
                                robot.ballsensor.argb() > -80000000 && robot.ballsensor.argb() < 0) {
                            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                        } else if (robot.ballsensor.argb() < 0 && robot.ballsensor.argb() > -100000000) {
                            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                        } else {
                            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                        }
                    } else if (runtime.seconds() > 80 && runtime.seconds() < 85) {
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
                    } else if (runtime.seconds() >= 85 && runtime.seconds() < 90) {
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
                    } else if (runtime.seconds() >= 90 && runtime.seconds() < 110) {
                        if (robot.blocksensor.argb() > 0 && robot.blocksensor.argb() < 16000000 ||
                                robot.blocksensor.argb() > 17000000 && robot.blocksensor.argb() < 33000000 ||
                                robot.blocksensor.argb() > 34000000 && robot.blocksensor.argb() < 50000000 ||
                                robot.ballsensor.argb() > -80000000 && robot.ballsensor.argb() < 0) {
                            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                        } else if (robot.ballsensor.argb() < 0 && robot.ballsensor.argb() > -100000000) {
                            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                        } else {
                            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                        }
                    } else if (runtime.seconds() >= 110 && runtime.seconds() < 115) {
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
                    } else if (runtime.seconds() >= 115 && runtime.seconds() < 120) {
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
                    } else {
                        robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    } */



//------------------------------------------------------------------------------------

                    if (gamepad1.left_bumper && gamepad1.right_bumper) {
                        // this loop is to reset our lifter motor IF NEEDED. SHOULD NEVER HAPPEN ON ACCIDENT
                        robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        if (gamepad1.right_trigger == 1) {
                            robot.lifter.setPower(0.6);
                        } else if (gamepad1.left_trigger == 1) {
                            robot.lifter.setPower(-0.6);
                        } else {
                            robot.lifter.setPower(0);
                        }
                        if (gamepad1.dpad_right) {
                            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }
                    } else {
                        if (!gamepad2.left_bumper && gamepad2.a) {
                            robot.lifter.setTargetPosition(robot.bottom);
                        } else if (!gamepad2.left_bumper && gamepad2.b) {
                            robot.lifter.setTargetPosition(robot.layer2);
                        } else if (gamepad2.left_bumper && gamepad2.a) {
                            robot.lifter.setTargetPosition(robot.layer3);
                        } else if (gamepad2.left_bumper && gamepad2.b) {
                            robot.lifter.setTargetPosition(robot.TipTop);
                        } else if (gamepad2.left_bumper && gamepad2.right_bumper) {
                            robot.lifter.setTargetPosition(-10);
                        }


                        if (robot.lifter.getCurrentPosition() <= (robot.lifter.getTargetPosition() - 15) || robot.lifter.getCurrentPosition() >= (robot.lifter.getTargetPosition() + 15)) {
                            robot.lifter.setPower(0.9);
                        } else {
                            robot.lifter.setPower(0);
                        }
                    }

//------------------------------------------------------------------------------------

                    double powermotor = 0.35;

                    if (gamepad1.right_bumper) {
                        powermotor = 1;
                    } else if (gamepad1.left_bumper) {
                        powermotor = 0.25;
                    }

//------------------------------------------------------------------------------------

                    if (driver == true) {
                        rearleft = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
                        frontleft = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
                        frontright = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
                        rearright = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;

                        robot.frontLeftMotor.setPower(frontleft);
                        robot.rearLeftMotor.setPower(rearleft);
                        robot.rearRightMotor.setPower(rearright);
                        robot.frontRightMotor.setPower(frontright);

                        if (gamepad1.left_trigger == 1 && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
                            rearleft = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor * 1.07;
                            rearright = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor * 1.07;
                            robot.rearLeftMotor.setPower(rearleft);
                            robot.rearRightMotor.setPower(rearright);
                        }

                        if (gamepad2.left_bumper && gamepad2.dpad_up) {
                            robot.elementclamp1.setPosition(.5);
                            robot.elementclamp2.setPosition(.5);
                        } else {
                            robot.elementclamp1.setPosition(0);
                            robot.elementclamp2.setPosition(0);
                        }

                        if (!gamepad2.left_bumper && gamepad2.dpad_down) {
                            robot.elementarm.setPosition(0);
                        } else if (!gamepad2.left_bumper && gamepad2.dpad_up && !gamepad2.right_bumper) {
                            robot.elementarm.setPosition(0.15);
                        } else if (gamepad2.right_bumper && gamepad2.dpad_up) {
                            robot.elementarm.setPosition(0.4);
                        } else if (gamepad1.a) {
                            robot.elementarm.setPosition(0);
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

                        if (gamepad1.left_trigger == 1) {
                            rearleft = (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powermotor * 1.07;
                            rearright = (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powermotor * 1.07;
                            robot.rearLeftMotor.setPower(rearleft);
                            robot.rearRightMotor.setPower(rearright);
                        }

                        if (gamepad2.left_bumper && gamepad2.dpad_up) {
                            robot.elementclamp1.setPosition(.5);
                        } else if (gamepad2.left_bumper && gamepad2.dpad_down && !gamepad2.right_bumper) {
                            robot.elementclamp1.setPosition(0);
                        }

                        if (!gamepad2.left_bumper && gamepad2.dpad_down) {
                            robot.elementarm.setPosition(0);
                        } else if (!gamepad2.left_bumper && gamepad2.dpad_up && !gamepad2.right_bumper) {
                            robot.elementarm.setPosition(0.15);
                        } else if (gamepad2.right_bumper && gamepad2.dpad_up) {
                            robot.elementarm.setPosition(0.4);
                        } else if (gamepad1.a) {
                            robot.elementarm.setPosition(0);
                        }
                    }
                }
            }

                if (dillon) {

                    double powermotor = 0.35;

                    if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                        powermotor = 1;
                    } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                        powermotor = 0.25;
                    }

                    rearleft = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
                    frontleft = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * powermotor;
                    frontright = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;
                    rearright = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * powermotor;

                    robot.frontLeftMotor.setPower(frontleft);
                    robot.rearLeftMotor.setPower(rearleft);
                    robot.rearRightMotor.setPower(rearright);
                    robot.frontRightMotor.setPower(frontright);

                    if (robot.lifter.getCurrentPosition() < 50) {
                        if (gamepad1.right_trigger == 1) {
                            robot.intakemotor.setPower(-.95);
                        } else if (gamepad1.left_trigger == 1) {
                            robot.intakemotor.setPower(.95);
                        } else {
                            robot.intakemotor.setPower(0);
                        }
                    } else {
                        robot.intakemotor.setPower(0);
                    }

//------------------------------------------------------------------------------------

                    if (gamepad1.y && !gamepad1.left_bumper) {
                        while (gamepad1.y) {
                            robot.duckspin.setPower(0.8);
                            robot.duckspinblue.setPower(0.8);
                        }
                        robot.duckspin.setPower(-.2);
                        robot.duckspinblue.setPower(-.2);
                    } else {
                        robot.duckspin.setPower(0);
                        robot.duckspinblue.setPower(0);
                    }

//------------------------------------------------------------------------------------

                    if (gamepad1.x && !gamepad1.left_bumper) {
                        robot.lightsaber.setPosition(0.4);
                    } else {
                        robot.lightsaber.setPosition(0);
                    }

                    if (gamepad2.left_bumper && gamepad2.dpad_up) {
                        robot.elementclamp1.setPosition(.5);
                    } else if (gamepad2.left_bumper && gamepad2.dpad_down && !gamepad2.right_bumper) {
                        robot.elementclamp1.setPosition(0);
                    }

                    if (!gamepad2.left_bumper && gamepad2.dpad_down) {
                        robot.elementarm.setPosition(0);
                    } else if (!gamepad2.left_bumper && gamepad2.dpad_up && !gamepad2.right_bumper) {
                        robot.elementarm.setPosition(0.15);
                    } else if (gamepad2.right_bumper && gamepad2.dpad_up) {
                        robot.elementarm.setPosition(0.4);
                    } else if (gamepad1.a) {
                        robot.elementarm.setPosition(0);
                    }

//------------------------------------------------------------------------------------

                    if (gamepad1.dpad_left) {
                        robot.duckextend.setPower(.9);
                    } else if (gamepad1.dpad_right) {
                        robot.duckextend.setPower(-.9);
                    } else {
                        robot.duckextend.setPower(0);
                    }

//------------------------------------------------------------------------------------

                        if (gamepad1.left_bumper && gamepad1.right_bumper) {
                            // this loop is to reset our lifter motor IF NEEDED. SHOULD NEVER HAPPEN ON ACCIDENT
                            robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            if (gamepad1.right_trigger == 1) {
                                robot.lifter.setPower(0.6);
                            } else if (gamepad1.left_trigger == 1) {
                                robot.lifter.setPower(-0.6);
                            } else {
                                robot.lifter.setPower(0);
                            }
                            if (gamepad1.dpad_right) {
                                robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            }
                        } else {
                            if (!gamepad1.left_bumper && gamepad1.a) {
                                robot.lifter.setTargetPosition(robot.bottom);
                            } else if (!gamepad1.left_bumper && gamepad1.b) {
                                robot.lifter.setTargetPosition(robot.layer2);
                            } else if (gamepad1.left_bumper && gamepad1.a) {
                                robot.lifter.setTargetPosition(robot.layer3);
                            } else if (gamepad1.left_bumper && gamepad1.b) {
                                robot.lifter.setTargetPosition(robot.TipTop);
                            } else if (gamepad1.a && gamepad1.b) {
                                robot.lifter.setTargetPosition(-10);
                            }
                        }


                            if (robot.lifter.getCurrentPosition() <= (robot.lifter.getTargetPosition() - 15) || robot.lifter.getCurrentPosition() >= (robot.lifter.getTargetPosition() + 15)) {
                                robot.lifter.setPower(0.9);
                            } else {
                                robot.lifter.setPower(0);
                            }

                            /*if (runtime.seconds() > 0 && runtime.seconds() <= 80) {
                                if (robot.blocksensor.argb() > 0 && robot.blocksensor.argb() < 16000000 ||
                                        robot.blocksensor.argb() > 17000000 && robot.blocksensor.argb() < 33000000 ||
                                        robot.blocksensor.argb() > 34000000 && robot.blocksensor.argb() < 50000000 ||
                                        robot.ballsensor.argb() > -33000000 && robot.ballsensor.argb() < 0 ||
                                        robot.ballsensor.argb() > -85000000 && robot.ballsensor.argb() < -40000000 ||
                                        robot.ballsensor.argb() > -370000000 && robot.ballsensor.argb() < -320000000 ||
                                        robot.ballsensor.argb() < -900000000) {
                                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                                } else if (robot.ballsensor.argb() < 0 && robot.ballsensor.argb() > -100000000) {
                                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                                } else {
                                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                                }
                            } else if (runtime.seconds() > 80 && runtime.seconds() < 85) {
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
                            } else if (runtime.seconds() >= 85 && runtime.seconds() < 90) {
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
                            } else if (runtime.seconds() >= 90 && runtime.seconds() < 110) {
                                if (robot.blocksensor.argb() > 0 && robot.blocksensor.argb() < 16000000 ||
                                        robot.blocksensor.argb() > 17000000 && robot.blocksensor.argb() < 33000000 ||
                                        robot.blocksensor.argb() > 34000000 && robot.blocksensor.argb() < 50000000 ||
                                        robot.ballsensor.argb() > -80000000 && robot.ballsensor.argb() < 0) {
                                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                                } else if (robot.ballsensor.argb() < 0 && robot.ballsensor.argb() > -100000000) {
                                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                                } else {
                                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                                }
                            } else if (runtime.seconds() >= 110 && runtime.seconds() < 115) {
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
                            } else if (runtime.seconds() >= 115 && runtime.seconds() < 120) {
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
                            } else {
                                robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                            }*/

                    }


//------------------------------------------------------------------------------------

                    // Show the elapsed game time and wheel power.
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("leftWheel", robot.frontRightMotor.getCurrentPosition());
                    telemetry.addData("rightWheel", robot.rearLeftMotor.getCurrentPosition());
                    telemetry.addData("middleWheel", robot.frontLeftMotor.getCurrentPosition());
                    telemetry.addData("duckExtend", robot.duckextend.getCurrentPosition());
                    telemetry.addData("DR", robot.distanceR.getDistance(DistanceUnit.INCH));
                    telemetry.addData("DL", robot.distanceL.getDistance(DistanceUnit.INCH));
                    telemetry.addData("BS-R", robot.blocksensor.red());
                    telemetry.addData("BS-G", robot.blocksensor.green());
                    telemetry.addData("BS-B", robot.blocksensor.blue());
                    telemetry.addData("BS-ARGB", robot.blocksensor.argb());
                    telemetry.addData("Balls-R", robot.ballsensor.red());
                    telemetry.addData("Balls-G", robot.ballsensor.green());
                    telemetry.addData("Balls-B", robot.ballsensor.blue());
                    telemetry.addData("Balls-ARGB", robot.ballsensor.argb());
                    telemetry.addData("lifterCP", robot.lifter.getCurrentPosition());
                    telemetry.addData("lifterTP", robot.lifter.getTargetPosition());
                    telemetry.addData("Lift Power", robot.lifter.getPower());
                    telemetry.update();

//------------------------------------------------------------------------------------


                }
            }
        }
