package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Utilities.angleWrap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
	private autoraonJava2 main;
	private Telemetry telemetry;
	double Kp;
	double Ki;
	double Kd;
	double integralSum;
	double lastError;
	ElapsedTime timer;
	MultipleTelemetry tmt;/* = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());*/

	public PIDController(autoraonJava2 main, Telemetry telemetry) {
		this.main = main;
		this.telemetry = telemetry;

		tmt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

}
