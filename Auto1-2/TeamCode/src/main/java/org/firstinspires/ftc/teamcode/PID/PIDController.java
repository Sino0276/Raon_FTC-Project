package org.firstinspires.ftc.teamcode.PID;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autoraonJava1;

public class PIDController {
	private autoraonJava1 main;
	double Kp;
	double Ki;
	double Kd;
	double integralSum;
	double lastError;
	ElapsedTime timer;
	MultipleTelemetry tmt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

	public PIDController(autoraonJava1 main) {
		this.main = main;
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

		tmt.addData("목표 IMU Angle", reference);
		tmt.addData("현재 IMU Angle", now);

		error = angleWrap(reference - now);
		derivative = (error - lastError) / timer.milliseconds();
		integralSum += error * timer.milliseconds();
		out = -(Kp * error + Ki * integralSum + Kd * derivative);

		lastError = error;
		timer.reset();

		tmt.update();

		return out;
	}

	public void resetPID() {
		// PID변수
		integralSum = 0;
		lastError = 0;
		timer.reset();
	}

	private double angleWrap(double wrappingAngle) {
		while (wrappingAngle > 180) {
			wrappingAngle = wrappingAngle - 360;
		}
		while (wrappingAngle < -180) {
			wrappingAngle = wrappingAngle + 360;
		}
		return wrappingAngle;
	}

}
