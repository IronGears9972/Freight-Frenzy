package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled

@TeleOp(name = "GearsTeleOpCapital", group = "Linear Opmode")
public class Tele_op_capital extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_20_21 robot = new Hardware_20_21(); // use the class created to define a robot's hardware

    // Minimum rotational position

    private ElapsedTime runtime = new ElapsedTime();


    private enum Shoot3 {Idle,Fork1,Conveyon,Fork2,Kick, Conveydone};

    private enum Shoot1 {Idlea,Forka1,Conveyona,Kicka,Conveydonea};


    private Shoot3SM shoot3SM=new Shoot3SM();

    private Shoot1SM shoot1SM=new Shoot1SM();




   /*
    Code to run ONCE when the driver hits INIT
   */

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
            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */

            // Send telemetry message to signify robot waiting;





            if (gamepad2.dpad_left) {
                robot.drivestraight2(-10,0.6);
                robot.drivestrafe(-5,0.1);

                shoot1SM.Starta();

            }



//------------------------------------------------------------------------------------


            if (gamepad1.right_stick_button) {
                robot.intakemotor.setPower(0.95);
                robot.intakeservo.setPower(0.8);
            } else if (gamepad1.left_stick_button) {
                robot.intakemotor.setPower(-0.95);
                robot.intakeservo.setPower(-0.8);
            } else {
                robot.intakemotor.setPower(0);
                robot.intakeservo.setPower(0);
            }


//------------------------------------------------------------------------------------


            if (gamepad1.y) {

                robot.launcher1.setPower(0.95);

            } else if (gamepad2.b) {

                robot.launcher1.setPower(0);
            }//else if (gamepad2.right_stick_button) {

               // robot.launcher1.setPower(0.69);
           // }else if (gamepad2.left_stick_button) {

                //robot.launcher1.setPower(0.6);
            //}


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


            if (gamepad1.x) {

                shoot3SM.Start();

            }

            shoot3SM.update();

            // One button for forks and kicker

            if (gamepad1.a) {

                shoot1SM.Starta();

            }

            shoot1SM.update();


//------------------------------------------------------------------------------------


            if (gamepad1.right_bumper) {
                robot.conveyor.setPower(-0.95);
            } else if (gamepad1.left_bumper) {
                robot.conveyor.setPower(0.25);
            } else if (shoot3SM.state== Shoot3.Idle&&shoot1SM.statea== Shoot1.Idlea){
                robot.conveyor.setPower(0);
            }


//------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------

            /*

            if (robot.digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                robot.robotsleep(0);
                sleep(10000);
            }

             */

//------------------------------------------------------------------------------------


            double powermotor = 0.95;

            //if (gamepad1.right_bumper) {
             //   powermotor = 0.75;
            //} else if (gamepad1.left_bumper) {
              //  powermotor = 0.50;
           // }


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

    class Shoot1SM{

        private Shoot1 statea= Shoot1.Idlea;

        private ElapsedTime timer=new ElapsedTime();

        public void update(){

        switch(statea){

            case Idlea:
                break;


            case Forka1:
                if (timer.milliseconds()>450){

                    statea= Shoot1.Conveyona;
                    robot.conveyor.setPower(0.25);
                    timer.reset();

                }

                break;

            case Conveyona:
                if (timer.milliseconds()>150){

                    statea= Shoot1.Kicka;
                    robot.forks.setPosition(0.30);
                    robot.kicker.setPosition(0.5);
                    timer.reset();

                }

                break;

            case Kicka:
                if (timer.milliseconds()>450){

                    statea= Shoot1.Conveydonea;
                    robot.conveyor.setPower(0.25);
                    timer.reset();

                }

                break;
            case Conveydonea:
                if (timer.milliseconds()>250){

                    statea= Shoot1.Idlea;
                    robot.forks.setPosition(0.05);
                    robot.kicker.setPosition(1);
                    robot.conveyor.setPower(0);
                    timer.reset();

                }

                break;


        }

    }

        public void Starta(){

        if (statea!= Shoot1.Idlea){
            return;
        }

        statea= Shoot1.Forka1;
        robot.forks.setPosition(0.30);
        timer.reset();

    }

    }


    class Shoot3SM {

        private Shoot3 state= Shoot3.Idle;

        private ElapsedTime timer=new ElapsedTime();

        public void update(){

            switch(state){

                case Idle:
                    break;

                case Fork1:
                    if (timer.milliseconds()>450){

                        state= Shoot3.Conveyon;
                        robot.conveyor.setPower(0.95);
                        timer.reset();

                    }

                    break;

                case Conveyon:
                    if (timer.milliseconds()>150){

                        state= Shoot3.Fork2;
                        robot.forks.setPosition(0.30);
                        timer.reset();

                    }

                    break;

                case Fork2:
                    if (timer.milliseconds()>450){

                        state= Shoot3.Kick;
                        robot.kicker.setPosition(0.7);
                        timer.reset();

                    }

                    break;

                case Kick:
                    if (timer.milliseconds()>250){

                        state= Shoot3.Conveydone;
                        robot.conveyor.setPower(0.95);
                        robot.kicker.setPosition(0.5);
                        timer.reset();

                    }

                    break;
                case Conveydone:
                    if (timer.milliseconds()>1150){

                        state= Shoot3.Idle;
                        robot.forks.setPosition(0.05);
                        robot.kicker.setPosition(1);
                        robot.conveyor.setPower(0);
                        timer.reset();

                    }

                    break;


            }

        }

        public void Start(){

            if (state!= Shoot3.Idle){
                return;
            }

            state= Shoot3.Fork1;
            robot.forks.setPosition(0.24);
            timer.reset();

        }

    }

}
