package org.firstinspires.ftc.teamcode.teamcode.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous(name="Blue Side", group="Pushbot")
public class AutoBlue extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_21_22 robot = new Hardware_21_22(); // use the class created to define a robot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    int layer;
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    double DR;
    double DL;
    int targetL = 0;
    int dist = 0;
    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;

        robot.init(hardwareMap, this);
        robot.duckextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.duckextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.duckextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("CP", robot.duckextend.getCurrentPosition());
        telemetry.update();
        sleep(1);
        OpRunENCODERZero();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            //-------------------------------------------------------------------------------------

            // This part of the program extends the duck spinner and starts spinning it

            robot.duckextend.setTargetPosition(1090);
            robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.duckextend.getCurrentPosition() < 1090) {
                telemetry.addData("CP", robot.duckextend.getCurrentPosition());
                telemetry.update();
                robot.duckextend.setPower(-0.9);
                sleep(1);
            }
            sleep(100);
            robot.duckextend.setPower(0);

            // The spinning ends here, and the distance sensors read what they can see

            for (int x = 0; x < 10; ++x) {
                DR = DR + robot.distanceR.getDistance(DistanceUnit.INCH);
                DL = DL + robot.distanceL.getDistance(DistanceUnit.INCH);
                telemetry.addData("DR", DR);
                telemetry.addData("DL", DL);
                telemetry.update();
                sleep(50);
            }

            // Averages the two distance readings to account for any misreadings, then
            DR = DR / 10;
            DL = DL / 10;

            if (DL > 17 && DL < 24) {
                layer = 2;
            } else if (DR > 17 && DR < 24) {
                layer = 3;
            } else {
                layer = 1;
            }

            telemetry.addData("laYER", layer);
            telemetry.update();

            robot.duckspinblue.setPower(0.8);
            sleep(2500);
            robot.duckspinblue.setPower(0);

            //-------------------------------------------------------------------------------------

            robot.duckextend.setTargetPosition(10);
            robot.duckextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.duckextend.setPower(0.8);
            sleep(1000);
            robot.elementarm.setPosition(0.01);
            robot.elementclamp.setPosition(0.5);

            sleep(500);

            robot.drivestraight(9, 0.25);
            robot.robotsleep(0);
            sleep(500);

            if (layer == 3) {
                robot.drivestrafe(4, 0.2, "N/A");
                dist = 4;
            } else if (layer == 2) {
                robot.drivestrafe(-4, 0.2, "N/A");
                dist = -4;
            } else if (layer == 1) {
                robot.drivestrafe(-12, 0.2, "N/A");
                dist = -12;
            }
            robot.cargolights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            robot.robotsleep(0);
            sleep(500);
            robot.duckextend.setPower(0);
            //-------------------------------------------------------------------------------------

            robot.drivestraight(10, 0.15);
            sleep(250);
            robot.elementclamp.setPosition(0);
            sleep(750);
            robot.elementarm.setPosition(0.55);
            sleep(500);

            //-------------------------------------------------------------------------------------

            robot.drivestrafe(-1*(26 + dist), 0.2, "N/A");
            robot.robotsleep(0);
            sleep(250);

            //-------------------------------------------------------------------------------------

            robot.robotsleep(0);
            if (layer == 1) {
                targetL = robot.bottom;
            } else if (layer == 2) {
                targetL = robot.layer2;
            } else if (layer == 3) {
                targetL = robot.layer3;
            }

            while (robot.lifter.getCurrentPosition() < targetL) {
                robot.lifter.setPower(0.95);
                telemetry.addData("Current-Lift", robot.lifter.getCurrentPosition());
                telemetry.addData("Target-Lift", targetL);
                telemetry.update();
                sleep(1);
            }
            robot.lifter.setPower(0);

            robot.drivestraight(2.5, 0.15);
            robot.lightsaber.setPosition(0.5);
            sleep(500);

            //-------------------------------------------------------------------------------------

            robot.lifter.setTargetPosition(0);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(-0.95);
            robot.drivestraight(-29.5, 0.3);
            robot.lightsaber.setPosition(0);
            robot.robotsleep(0);
            sleep(500);
            robot.drivestrafe(-56, 0.2, "S1");
            robot.elementarm.setPosition(0.05);
            robot.lifter.setPower(0);
            sleep(500);

            //-------------------------------------------------------------------------------------

            //OpRunTelemetryZero();
            break;
        }
    }

    private void OpRunENCODERZero() {
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //              ^ Starts all Encoders
    /*
    private void OpRunTelemetryZero() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
        telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
        telemetry.addData("lifter", " %.0d",robot.lifter.getCurrentPosition());
        telemetry.update();
    }
     */

    //              ^ Outputs text so we can read data
}
