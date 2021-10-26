package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="4 Rings", group="Pushbot")

public class Autonomous4Again extends LinearOpMode {

    Hardware_21_22 robot = new Hardware_21_22();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

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


        while (!opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical left encoder position", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.rearRightMotor.getCurrentPosition());
            telemetry.addData("intake", " %d", robot.intakemotor.getCurrentPosition());
            telemetry.addData("launcher1", " %.0f", robot.launcher1.getVelocity() / 28 * 60);
            telemetry.addData("DSRearLeft", String.format("%.01f in", robot.DSRearLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftBack", String.format("%.01f in", robot.DSLeftBack.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSLeftFront", String.format("%.01f in", robot.DSLeftFront.getDistance(DistanceUnit.INCH)));
            telemetry.addData("DSRearRight", String.format("%.01f in", robot.DSRearRight.getDistance(DistanceUnit.INCH)));
            Orientation angels = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("imu , first/second/third", "%.1f %.1f %.1f", angels.firstAngle, angels.secondAngle, angels.thirdAngle);
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.update();



        }

        robot.wobblehand2.setPosition(0.35);
        robot.lightsaber.setPosition(0.96);
        robot.drivestraight(-25, 0.3);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestrafe(8,0.3);
        robot.lightsaber.setPosition(0.32);

        robot.robotsleep(0);
        sleep(100);

        robot.launcher1.setPower(0.90);
        robot.lightsaber.setPosition(0.32);

        robot.drivestraight(-40, 0.3);
        robot.lightsaber.setPosition(0.96);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestrafe(-19,0.3);

        robot.robotsleep(0);
        sleep(100);

        //robot.launcher1.setPower(0.95);
        //sleep(1400);

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
        robot.forks.setPosition(0.28);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(-0.95);
        sleep(950);


        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);
        robot.forks.setPosition(0.05);
        robot.kicker.setPosition(1);
        robot.intakemotor.setPower(-0.95);
        robot.intakeservo.setPower(-0.8);

        robot.drivestraight(-67, 0.5);

        robot.robotsleep(0);
        sleep(200);

        robot.wobble2.setPosition(0.15);
        sleep(600);

        robot.drivestrafe(22,0.3);
        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);

        robot.wobblehand2.setPosition(0.85);
        robot.conveyor.setPower(-0.55);
        sleep(400);

        robot.drivestrafe(-50,0.4);

        robot.robotsleep(0);
        robot.conveyor.setPower(0);
        sleep(200);

        robot.wobblekeep.setPosition(1);

        robot.drivestraight(132, 0.6);

        robot.robotsleep(0);
        robot.forks.setPosition(0.28);
        sleep(200);

        robot.drivestrafe2(36,0.10);

        robot.wobblehand2.setPosition(0.34);
        robot.robotsleep(0);
        sleep(450);

        /*

        robot.drivestrafe(26,0.3);

        robot.intakemotor.setPower(-0.95);
        robot.intakeservo.setPower(-0.8);

        robot.launcher1.setPower(0.55);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestraightleft(-36, 0.3);//50

        robot.robotsleep(0);//
        sleep(1000);//600

        robot.drivestraight(-6, 0.3);

        robot.robotsleep(0);//
        sleep(1200);//600

        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(0.8);
        sleep(500);//700

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);
        robot.wobblekeep.setPosition(0.4);


        //robot.launcher1.setPower(0.75);
        //sleep(1350);


        robot.launcher1.setPower(0.55);


        robot.conveyor.setPower(-0.95);
        //robot.forks.setPosition(0.24);
        sleep(275);


        //robot.kicker.setPosition(0.6);
        //sleep(175);

        robot.conveyor.setPower(0.35);
        robot.forks.setPosition(0.24);
        //robot.kicker.setPosition(1);
        sleep(400);

        robot.conveyor.setPower(-0.95);
        sleep(350);
        robot.forks.setPosition(0.29);
        sleep(300);

        robot.kicker.setPosition(0.6);
        sleep(50);

        robot.conveyor.setPower(-0.95);
        sleep(950);

        robot.conveyor.setPower(0);
        robot.forks.setPosition(0.05);
        robot.kicker.setPosition(1);


        robot.intakemotor.setPower(-0.95);
        robot.intakeservo.setPower(-0.8);

        robot.robotsleep(0);
        sleep(100);

        robot.drivestraightleft(-36, 0.3);//50

        robot.robotsleep(0);//
        sleep(1000);//600

        robot.drivestraight(-6, 0.3);

        robot.robotsleep(0);//
        sleep(1200);//600

        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(0.8);
        sleep(500);//700

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);



       */

        robot.robotsleep(0);
        sleep(200);

        robot.drivestraight(-130, 0.6);

        robot.wobblekeep.setPosition(0.4);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(42,0.4);

        robot.wobblehand2.setPosition(0.85);
        sleep(750);

        robot.drivestrafe(-2,0.8);

        robot.drivestraight(46, 0.55);

        robot.robotsleep(0);
        sleep(200);



    }

}
