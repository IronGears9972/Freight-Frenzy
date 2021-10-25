package org.firstinspires.ftc.teamcode.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware_20_21;


@TeleOp(name = "GearsTeleOp", group = "Linear Opmode")
public class Tele_op_ritw extends LinearOpMode {

    Hardware_20_21 robot = new Hardware_20_21();

    private ElapsedTime runtime = new ElapsedTime();

    private enum ShootPS {IdlePS,Fork1PS,ConveyonPS,Fork2PS,KickPS, ConveydonePS};

    private enum Shoot3 {Idle,Fork1,Conveyon,Fork2,Kick, Conveydone};

    private enum Shoot1 {Idlea,Forka1,Conveyona,Kicka,Conveydonea};



    private ShootPSSM shootPSSM=new ShootPSSM();

    private Shoot3SM shoot3SM=new Shoot3SM();

    private Shoot1SM shoot1SM=new Shoot1SM();


    public void runOpMode() {


        telemetry.addData("Say", "Hello Iron Gears");//
        telemetry.update();


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
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        robot.forks.setPosition(0.05);
        robot.kicker.setPosition(1);


        while (opModeIsActive()) {

            if (gamepad1.y) {
                robot.launcher1.setPower(0.5);
                sleep(1000);

                robot.robotsleep(0);
                sleep(200);

                robot.conveyor.setPower(-0.95);

                robot.robotsleep(0);
                sleep(200);

                robot.drivestrafe(-8,0.2);
                robot.robotsleep(0);
                sleep(200);

                robot.forks.setPosition(0.24);
                sleep(200);

                robot.conveyor.setPower(-0.95);
                sleep(550);

                robot.drivestrafe(-8,0.2);
                robot.robotsleep(0);
                sleep(400);

                robot.forks.setPosition(0.28);
                robot.kicker.setPosition(0.7);
                sleep(400);

                robot.launcher1.setPower(0);

            }



//------------------------------------------------------------------------------------


            if (gamepad2.dpad_up) {
                robot.intakemotor.setPower(0.95);
                robot.intakeservo.setPower(0.8);
            } else if (gamepad2.dpad_down) {
                robot.intakemotor.setPower(-0.95);
                robot.intakeservo.setPower(-0.8);
            } else {
                robot.intakemotor.setPower(0);
                robot.intakeservo.setPower(0);
            }


//------------------------------------------------------------------------------------


            if (gamepad2.y) {

                robot.launcher1.setPower(0.90);

            } else if (gamepad2.b) {

                robot.launcher1.setPower(0);
            }else if (gamepad2.right_stick_button) {

                robot.launcher1.setPower(0.69);
            }else if (gamepad2.left_stick_button) {

                robot.launcher1.setPower(0.50);
            }


//------------------------------------------------------------------------------------


            if (gamepad1.dpad_right) {
                robot.wobblehand.setPosition(0);


            } else if (gamepad1.dpad_left) {
                robot.wobblehand.setPosition(1);

            }


            if (gamepad1.dpad_down) {
                robot.wobble.setPosition(1);



            } else if (gamepad1.dpad_up) {
                robot.wobble.setPosition(0);


            }


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


            if (gamepad1.a) {
                robot.wobblehand2.setPosition(0.35);
                sleep(500);
                robot.wobble2.setPosition(1);
            }


//------------------------------------------------------------------------------------



            if (gamepad1.left_stick_button) {
                robot.lightsaber.setPosition(0.32);
            }else if (gamepad1.right_stick_button) {
                robot.lightsaber.setPosition(0.96);

            }


//------------------------------------------------------------------------------------




            if (gamepad2.x) {

                shoot3SM.Start();

            }

            shoot3SM.update();


            if (gamepad2.a) {

                shoot1SM.Starta();

            }

            shoot1SM.update();


//------------------------------------------------------------------------------------


            if (gamepad2.right_bumper) {
                robot.conveyor.setPower(-0.95);
            } else if (gamepad2.left_bumper) {
                robot.conveyor.setPower(0.25);
            } else if (shoot3SM.state==Shoot3.Idle&&shoot1SM.statea==Shoot1.Idlea){
                robot.conveyor.setPower(0);
            }


//------------------------------------------------------------------------------------


            double powermotor = 0.95;

            if (gamepad1.right_bumper) {
                powermotor = 0.30;
            } else if (gamepad1.left_bumper) {
                powermotor = 0.50;
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
            telemetry.addData("SM",shoot3SM.state.toString());
            telemetry.addData("SM",shoot1SM.statea.toString());
            telemetry.addData("Forks",robot.forks.getPosition());

            telemetry.addData("Alpha", robot.sensorColorL.alpha());
            telemetry.addData("Red  ", robot.sensorColorL.red());
            telemetry.addData("Green", robot.sensorColorL.green());
            telemetry.addData("Blue ", robot.sensorColorL.blue());

            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("frontRightMotor", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("rearLeftMotor", robot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("rearRightMotor", robot.rearRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftMotor", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("launcher1", " %.0f",robot.launcher1.getVelocity()/28*60);
            telemetry.update();


//------------------------------------------------------------------------------------

        }


    }

    class ShootPSSM {

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

        public void StartPS(){

            if (state!=ShootPS.IdlePS){
                return;
            }

            state=ShootPS.Fork1PS;
            robot.forks.setPosition(0.24);
            timer.reset();

        }

    }


        class Shoot1SM{

        private Shoot1 statea=Shoot1.Idlea;

        private ElapsedTime timer=new ElapsedTime();

        public void update(){

        switch(statea){

            case Idlea:
                break;


            case Forka1:
                if (timer.milliseconds()>200){

                    statea=Shoot1.Conveyona;
                    robot.forks.setPosition(0.28);
                    timer.reset();

                }

                break;

            case Conveyona:
                if (timer.milliseconds()>150){

                    statea=Shoot1.Kicka;
                    robot.conveyor.setPower(-0.65);
                    robot.kicker.setPosition(0.5);
                    timer.reset();

                }

                break;

            case Kicka:
                if (timer.milliseconds()>150){

                    statea=Shoot1.Conveydonea;
                    robot.conveyor.setPower(-0.65);
                    timer.reset();

                }

                break;
            case Conveydonea:
                if (timer.milliseconds()>50){

                    statea=Shoot1.Idlea;
                    robot.forks.setPosition(0.05);
                    robot.kicker.setPosition(1);
                    robot.conveyor.setPower(0);
                    timer.reset();

                }

                break;


        }

    }

        public void Starta(){

        if (statea!=Shoot1.Idlea){
            return;
        }

        statea=Shoot1.Forka1;
        robot.forks.setPosition(0.28);
        timer.reset();

    }

    }


    class Shoot3SM {

        private Shoot3 state=Shoot3.Idle;

        private ElapsedTime timer=new ElapsedTime();

        public void update(){

            switch(state){

                case Idle:
                    break;

                case Fork1:
                    if (timer.milliseconds()>450){

                        state=Shoot3.Conveyon;
                        robot.conveyor.setPower(-0.95);
                        timer.reset();

                    }

                    break;

                case Conveyon:
                    if (timer.milliseconds()>150){

                        state=Shoot3.Fork2;
                        robot.forks.setPosition(0.28);//0.3
                        timer.reset();

                    }

                    break;

                case Fork2:
                    if (timer.milliseconds()>450){

                        state=Shoot3.Kick;
                        robot.kicker.setPosition(0.7);
                        timer.reset();

                    }

                    break;

                case Kick:
                    if (timer.milliseconds()>250){

                        state=Shoot3.Conveydone;
                        robot.conveyor.setPower(-0.95);
                        robot.kicker.setPosition(0.5);
                        timer.reset();

                    }

                    break;
                case Conveydone:
                    if (timer.milliseconds()>1150){

                        state=Shoot3.Idle;
                        robot.forks.setPosition(0.05);
                        robot.kicker.setPosition(1);
                        robot.conveyor.setPower(0);
                        timer.reset();

                    }

                    break;


            }

        }

        public void Start(){

            if (state!=Shoot3.Idle){
                return;
            }

            state=Shoot3.Fork1;
            robot.forks.setPosition(0.24);
            timer.reset();

        }

    }

}
