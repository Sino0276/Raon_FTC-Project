package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUController {
	// 여기
	private autoraonJava2 main;
	private IMU imu;
	private Telemetry telemetry;
	private double yaw;


	public IMUController(autoraonJava2 main) {
		this.main = main;

		imu = main.imu;
		telemetry = main.telemetry;

		_initIMU();
	}

	private void _initIMU() { //                                                                     로고 방향                                                 USB포트 방향
		imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
		imu.resetYaw();
		telemetry.addData("IMU 초기화 됨", "시작 대기중");
	}

	public double getYaw() { return yaw; }

	public void callIMU() {
		YawPitchRollAngles orientation;
		AngularVelocity angularVelocity;

		orientation = imu.getRobotYawPitchRollAngles();
		angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
		yaw = orientation.getYaw(AngleUnit.DEGREES);
	}
}
