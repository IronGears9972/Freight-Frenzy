package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Carousel {
	public DcMotorEx motor;
	private LinearOpMode opMode;
	private final static double WHEELS_RATIO = 4 / 15;
	private final static double OPTIMAL_ANGULAR_VELOCITY = 23;
	private Automate autoDucks = new Automate();
	private ExecutorService executor;
	public boolean isAutoDucksInvoked = false;
	private final static double CAROUSEL_ROTATION_TIME = (3 * Math.PI / OPTIMAL_ANGULAR_VELOCITY) / WHEELS_RATIO;

	public Carousel(LinearOpMode _opMode) {
		opMode = _opMode;
		motor = opMode.hardwareMap.get(DcMotorEx.class, "duckspin");
		executor = Executors.newSingleThreadExecutor();
	}

	/**
	 * Передает на мотор мощность равную power
	 *
	 * @param power Сила подаваемая на мотор
	 */
	public void motion(double power) {
		motor.setPower(power);
	}

	/**
	 * Ставит скорость 1 оборот в секунду на мотор
	 * Аргумент speed уменьшен в два разя для соотвествия угловой скорости в реальных тестах
	 */
	public void startWheel(double speed) {
		motor.setVelocity(speed / 2, AngleUnit.RADIANS);
	}

	public void startWheel() {
		motor.setVelocity(OPTIMAL_ANGULAR_VELOCITY, AngleUnit.RADIANS);
	}

	public void stopWheel(double max) {
		double velocity = motor.getVelocity(AngleUnit.RADIANS);
		double sig = Math.signum(velocity);

		if (Math.abs(velocity) > max) {
			motor.setVelocity(-sig * Math.PI, AngleUnit.RADIANS);
			opMode.sleep(80);
		}
		motor.setVelocity(0);
	}
	public void autoDucks(){
		if (!isAutoDucksInvoked){
			executor.execute(autoDucks);
		}
	}
	public class Automate implements Runnable {
		ElapsedTime timer = new ElapsedTime();

		@Override
		public void run() {
			isAutoDucksInvoked = true;
			timer.reset();
			startWheel();
			while ((timer.milliseconds() < CAROUSEL_ROTATION_TIME) && opMode.opModeIsActive()) {}
			stopWheel(6.4);
			isAutoDucksInvoked = false;
		}
	}
}