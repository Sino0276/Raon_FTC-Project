package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilities.angleWrap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
	// 여기
	public autoraonJava2 main;
	private double Kp;
	private double Ki;
	private double Kd;
	private double integralSum;
	private double lastError;
	private ElapsedTime timer;
	// 여기
	public Telemetry telemetry;
	public MultipleTelemetry multiTelemetry;

	public PIDController(autoraonJava2 main) {
		this.main = main;

		telemetry = main.telemetry;
		multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		_initPID();
	}

	private void _initPID() {
		// PID변수
		Kp = PIDConstants.Kp;
		Ki = PIDConstants.Ki;
		Kd = PIDConstants.Kd;
		integralSum = 0;
		lastError = 0;
		timer = new ElapsedTime();
	}

	/**
	 * https://www.ctrlaltftc.com/the-pid-controller
	 */
	public double PIDControl(double reference, double now) {
		double error;
		double derivative;
		double out;

		multiTelemetry.addData("목표 IMU Angle", reference);
		multiTelemetry.addData("현재 IMU Angle", now);

		error = angleWrap(reference - now);
		derivative = (error - lastError) / timer.milliseconds();
		integralSum += error * timer.milliseconds();
		out = -(Kp * error + Ki * integralSum + Kd * derivative);

		lastError = error;
		timer.reset();

		multiTelemetry.update();

		return out;
	}

	public void resetPID() {
		// PID변수
		integralSum = 0;
		lastError = 0;
		timer.reset();
	}

}
