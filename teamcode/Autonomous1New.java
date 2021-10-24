package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Rings 1", group="Pushbot")
public class Autonomous1New extends LinearOpMode {

    Hardware_20_21 robot = new Hardware_20_21();
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        float hsvValues[] = {0F, 0F, 0F};


        final float values[] = hsvValues;



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
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();


        }


        robot.wobblehand2.setPosition(0.35);
        robot.lightsaber.setPosition(0.96);
        robot.drivestraight(-27, 0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(-10,0.3);

        robot.launcher1.setPower(0.50);
        sleep(1100);

        robot.launcher1.setPower(0.50);

        //robot.conveyor.setTargetPosition(4800);//300
        robot.conveyor.setPower(-0.95);
        //robot.kicker.setPosition(0.6);
        sleep(175);

        robot.launcher1.setPower(0);//
        robot.conveyor.setPower(0);
        sleep(50);//

        robot.forks.setPosition(0.24);
        sleep(375);

        robot.conveyor.setPower(-0.35);//-0.95
        //robot.kicker.setPosition(1);
        sleep(250);//400//200

        robot.conveyor.setPower(0);//-0.95
        robot.drivestrafe(3,0.3);//
        robot.forks.setPosition(0.05);//0.29
        //sleep(300);

        //robot.kicker.setPosition(0.6);
        //sleep(50);

        //robot.conveyor.setPower(-0.95);
        //sleep(950);


        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);

        //robot.forks.setPosition(0.05);
        //robot.kicker.setPosition(1);




        robot.intakemotor.setPower(-0.95);
        robot.intakeservo.setPower(-0.8);

        robot.drivestraight(-12, 0.3);
        sleep(500);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestraight(-26, 0.3);
        robot.intakemotor.setPower(0.95);
        robot.intakeservo.setPower(0.8);
        sleep(700);

        robot.intakemotor.setPower(0);
        robot.intakeservo.setPower(0);

        /*

        robot.launcher1.setPower(0.95);
        sleep(1100);

        robot.launcher1.setPower(0.95);




        robot.conveyor.setPower(-0.95);

        robot.forks.setPosition(0.30);
        sleep(500);

        robot.kicker.setPosition(0.6);
        sleep(175);






        robot.robotsleep(0);
        sleep(1000);

        robot.conveyor.setPower(0);
        robot.launcher1.setPower(0);


         */

        robot.drivestrafe(-4,0.3);//


        robot.launcher1.setPower(0.90);
        sleep(1350);


        robot.launcher1.setPower(0.90);


        robot.conveyor.setPower(-0.95);
        //robot.forks.setPosition(0.24);
        sleep(275);


        //robot.kicker.setPosition(0.6);
        //sleep(175);

        //robot.conveyor.setPower(0.35);
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
        sleep(1050);



        robot.drivestraight(-50, 0.3);
        robot.launcher1.setPower(0);
        robot.conveyor.setPower(0);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(-6,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.wobble2.setPosition(0.15);
        sleep(700);

        robot.wobblehand2.setPosition(0.85);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(-35,0.3);

        robot.robotsleep(0);
        sleep(200);

        robot.wobblekeep.setPosition(1);

        robot.drivestraight(111.5,0.6);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe2(36,0.10);

        robot.wobblehand2.setPosition(0.34);
        robot.robotsleep(0);
        sleep(450);

        robot.drivestraight(-104, 0.6);//0.5

        robot.wobblekeep.setPosition(0.4);

        robot.robotsleep(0);
        sleep(200);

        robot.drivestrafe(19, 0.4);

        robot.wobblehand2.setPosition(1);
        sleep(400);

        robot.wobble2.setPosition(0.25);

        robot.drivestraight(28, 0.7);

        robot.robotsleep(0);
        sleep(200);

    }

}
