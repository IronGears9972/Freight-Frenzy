/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="new", group="Pushbot")
public class Autonomous0State extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_21_22 robot = new Hardware_21_22(); // use the class created to define a robot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    int layer;
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;

        robot.init(hardwareMap, this);

        OpRunENCODERZero();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //-------------------------------------------------------------------------------------
        //extends extendable arms

        robot.drivestraight(1,0.6);
        robot.robotsleep(0);
        sleep(50);

        robot.barcodeL.setPower(0.9);
        robot.barcodeR.setPower(-0.9);
        robot.duckextend.setPower(0.9);
        sleep(2000);

        robot.barcodeL.setPower(0);
        robot.barcodeR.setPower(0);
        robot.duckextend.setPower(0);
        sleep(50);

        //-------------------------------------------------------------------------------------
        // spins ducks and reads
        robot.duckspin.setPower(1.0);
        sleep(4000);
        robot.duckspin.setPower(0);
        boolean L = robot.BarcodeLeft.getState();
        boolean R = robot.BarcodeRight.getState();
        if (L == true){
            layer = 2;
        } else if (R == true){
            layer = 3;
        } else
            layer = 1;
        sleep(50);
        telemetry.addData("layer", layer);

        //-------------------------------------------------------------------------------------
                //retracts extendable arms


        robot.barcodeL.setPower(-0.9);
        robot.barcodeR.setPower(0.9);
        robot.duckextend.setPower(-0.9);
        sleep(2000);

        robot.barcodeL.setPower(0);
        robot.barcodeR.setPower(0);
        robot.duckextend.setPower(0);
        sleep(50);

        //-------------------------------------------------------------------------------------

        robot.drivestrafe(30,0.6);
        robot.robotsleep(0);
        sleep(500);
        robot.drivestraight(30,0.6);
        robot.robotsleep(0);
        sleep(50);

        //-------------------------------------------------------------------------------------

        robot.lifter.setPower(0.8);
        sleep(1000);
        robot.lifter.setPower(0);
        robot.lightsaber.setPosition(0.3);
        sleep(100);
        robot.lightsaber.setPosition(0.95);
        robot.lifter.setPower(-0.8);
        sleep(1000);
        robot.lifter.setPower(0);

        //-------------------------------------------------------------------------------------
        // 1st big diversion begins here
        robot.trunangel(90,0.4);
        robot.drivestrafe(30,0.6);
        robot.drivestraight(50, 0.6);
        //if it ends here, then the robot stops as soon as it gets into the warehouse

        //-------------------------------------------------------------------------------------
        //optional step that parks the robot deeper in the warehouse, closer to the shared shipping hub
        robot.drivestrafe(-30,0.5);
        robot.drivestraight(20, 0.6);

        //-------------------------------------------------------------------------------------

        OpRunTelemetryZero();

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

     private void OpRunTelemetryZero() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
        telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
        telemetry.addData("lifter", " %.0f",robot.lifter.getCurrentPosition());
        telemetry.update();
    }
    //              ^ Outputs text so we can read data




    }


