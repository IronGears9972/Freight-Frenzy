package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware_20_21;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Disabled

@Autonomous(name="4 Rings dud", group="Pushbot")

public class Autonomous4 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_21_22 robot = new Hardware_21_22(); // use the class created to define a robot's hardware
    private ElapsedTime     runtime = new ElapsedTime();




    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        
        robot.init(hardwareMap, this);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //STEP 1
        // Drive to the ring and shot

        robot.wobblehand2.setPosition(0.35);
        robot.drivestraight(-27, 0.3);

        robot.robotsleep(0);
        sleep(200);
        //------------------------------------------------------------------------------------------
        //STEP 2
        // Drive turn and drop the wobble
        robot.drivestrafe(-10,0.3);

        robot.launcher1.setPower(0.80);
        sleep(1100);

        robot.launcher1.setPower(0.80);




        robot.forks.setPosition(0.24);
        sleep(250);

        robot.conveyor.setPower(0.95);
        robot.kicker.setPosition(0.6);
        sleep(275);

        robot.launcher1.setPower(0.70);
        robot.kicker.setPosition(1);
        sleep(600);

        robot.forks.setPosition(0.30);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(150);

        robot.conveyor.setPower(0.95);
        sleep(1350);

        robot.forks.setPosition(0.05);

        robot.kicker.setPosition(1);

        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);









        robot.drivestrafe(3,0.3);

        robot.intakemotor.setPower(-.95);
        robot.intakeservo.setPower(0.8);

        robot.drivestraight(-12, 0.3);
        sleep(1000);

        robot.robotsleep(0);
        sleep(1200);

        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(-0.8);
        sleep(700);

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);

        robot.launcher1.setPower(0.80);
        sleep(1100);

        robot.launcher1.setPower(0.80);

        robot.forks.setPosition(0.24);
        sleep(250);

        robot.conveyor.setPower(0.95);
        robot.kicker.setPosition(0.6);
        sleep(175);

        robot.kicker.setPosition(1);
        sleep(600);

        robot.forks.setPosition(0.30);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(0.95);
        sleep(2150);

        robot.forks.setPosition(0.05);

        robot.kicker.setPosition(1);

        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);






        robot.drivestraight(1, 0.3);

        robot.intakemotor.setPower(-.95);
        robot.intakeservo.setPower(0.8);

        robot.drivestraight(-12, 0.3);
        sleep(1000);

        robot.robotsleep(0);
        sleep(1200);

        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(-0.8);
        sleep(700);

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);

        robot.launcher1.setPower(0.80);
        sleep(1100);

        robot.launcher1.setPower(0.80);

        robot.forks.setPosition(0.24);
        sleep(250);

        robot.conveyor.setPower(0.95);
        robot.kicker.setPosition(0.6);
        sleep(175);

        robot.kicker.setPosition(1);
        sleep(600);

        robot.forks.setPosition(0.30);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(0.95);
        sleep(2150);

        robot.forks.setPosition(0.05);

        robot.kicker.setPosition(1);

        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);






        robot.robotsleep(0);
        sleep(200);

        robot.conveyor.setPower(0);
        robot.launcher1.setPower(0);






        robot.drivestraight(-72, 0.6);

        robot.robotsleep(0);
        sleep(200);

        robot.wobble2.setPosition(0.15);

        robot.drivestrafe(30,0.6);

        robot.wobblehand2.setPosition(0.85);
        sleep(250);

        robot.drivestraight(20, 1);





    }
}
