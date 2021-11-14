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

    private enum ShootPS {IdlePS, Fork1PS, ConveyonPS, Fork2PS, KickPS, ConveydonePS}

    ;

    private enum Shoot3 {Idle, Fork1, Conveyon, Fork2, Kick, Conveydone}

    ;

    private enum Shoot1 {Idlea, Forka1, Conveyona, Kicka, Conveydonea}

    ;

    int tele = 3;
    int TargetL = 0;


    //private ShootPSSM shootPSSM=new ShootPSSM();


    public void runOpMode() {


        telemetry.addData("Say", "Hello Iron Gears");
        telemetry.update();
/*
        while (!(opModeIsActive())){

        }
  */
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

            if (gamepad2.left_bumper && gamepad2.dpad_down) {
                robot.elementarm.setPosition(0);
            } else if (gamepad2.left_bumper && gamepad2.dpad_up) {
                robot.elementarm.setPosition(0.25);
            } else if (gamepad2.right_bumper && gamepad2.dpad_up) {
                robot.elementarm.setPosition(0.5);
            }

//------------------------------------------------------------------------------------

            if (gamepad2.dpad_left) {
                robot.duckextend.setPower(.9);
            } else if (gamepad2.dpad_right) {
                robot.duckextend.setPower(-.9);
            } else {
                robot.duckextend.setPower(0);

//------------------------------------------------------------------------------------


                if (!gamepad2.left_bumper && gamepad2.dpad_up) {
                    robot.elementclamp.setPosition(.5);
                } else if (!gamepad2.left_bumper && gamepad2.dpad_down) {
                    robot.elementclamp.setPosition(0);
                }


//------------------------------------------------------------------------------------
/*
                if (robot.blocksensor.alpha() > 1500 && robot.blocksensor.alpha() < 3500) {
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                }
                else if ((robot.blocksensor.alpha() > 3500)){
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
                }
                else {
                    robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                }
*/
                if (robot.blocksensor.argb() < 0){

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


//------------------------------------------------------------------------------------

            }


        }



/*  class ShootPSSM {
      private ShootPS state=ShootPS.IdlePS;
      private ElapsedTime timer=new ElapsedTime();
      public void update(){
          switch(state){
              case IdlePS:
                  break;
              case Fork1PS:
                  if (timer.milliseconds()>450){
                      state=ShootPS.ConveyonPS;
                      robot.conveyor.setPower(-0.95);
                      timer.reset();
                  }
                  break;
              case ConveyonPS:
                  if (timer.milliseconds()>150){
                      state=ShootPS.Fork2PS;
                      robot.drivestrafe(-6,0.2);
                      timer.reset();
                  }
                  break;
              case Fork2PS:
                  if (timer.milliseconds()>450){
                      state=ShootPS.KickPS;
                      robot.kicker.setPosition(0.7);
                      timer.reset();
                  }
                  break;
              case KickPS:
                  if (timer.milliseconds()>250){
                      state=ShootPS.ConveydonePS;
                      robot.conveyor.setPower(-0.95);
                      robot.kicker.setPosition(0.5);
                      timer.reset();
                  }
                  break;
              case ConveydonePS:
                  if (timer.milliseconds()>1150){
                      state=ShootPS.IdlePS;
                      robot.forks.setPosition(0.05);
                      robot.kicker.setPosition(1);
                      robot.conveyor.setPower(0);
                      timer.reset();
                  }
                  break;
          }
      }
      */


    }
}


