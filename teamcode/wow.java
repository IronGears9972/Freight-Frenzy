package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="wow", group="Pushbot")
public class wow extends LinearOpMode {

    Hardware_20_21 robot = new Hardware_20_21();
    private ElapsedTime     runtime = new ElapsedTime();



    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);

        OpRunENCODERZero();

        waitForStart();

        robot.drivestraight(-115,0.65);
        robot.robotsleep(100);

        robot.drivestrafe(30,0.95);
        robot.robotsleep(100);

        robot.drivestraight(30,0.95);
        robot.robotsleep(100);

        robot.drivestrafe(-30,0.95);
        robot.robotsleep(100);

        robot.drivestraight(-30,0.95);
        robot.robotsleep(100);

        robot.drivestrafe(30,0.95);
        robot.robotsleep(100);

        robot.drivestraight(30,0.95);
        robot.robotsleep(100);

        robot.drivestrafe(-30,0.95);
        robot.robotsleep(100);

        robot.drivestraight(115,0.95);
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
        sleep(700);

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
