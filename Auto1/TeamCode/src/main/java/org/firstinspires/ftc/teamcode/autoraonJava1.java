package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "autoraonJava1")
public class autoraonJava1 extends LinearOpMode {
	MultipleTelemetry tmt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

	private DcMotor left1;
	private DcMotor left2;
	private DcMotor right1;
	private DcMotor right2;
	private DcMotor arm_angle;
	private DcMotor arm_length;
	private Servo finger;
	private Servo hand;

	private IMU imu;
	double yaw;
	List<AprilTagDetection> myAprilTagDetections;
	boolean isParking;
	boolean isBasket;
	AprilTagDetection aprilTag;
	AprilTagProcessor myAprilTagProcessor;

	double COUNTS_PER_INCH;
	int armAngleRev;    // 팔이 한바퀴 돌기 위해 필요한 인코더 값

	double rx;

	boolean USE_WEBCAM;

	double power;
	double turnSpeed;
	double denominator;
	double Kp;
	double Ki;
	double Kd;
	double integralSum;
	double lastError;
	ElapsedTime timer;
	int sleepTime;

	private void _initHardWareMap() {
		left1 = hardwareMap.get(DcMotor.class, "left1");
		left2 = hardwareMap.get(DcMotor.class, "left2");
		right1 = hardwareMap.get(DcMotor.class, "right1");
		right2 = hardwareMap.get(DcMotor.class, "right2");
		imu = hardwareMap.get(IMU.class, "imu");
		arm_angle = hardwareMap.get(DcMotor.class, "arm_angle");
		arm_length = hardwareMap.get(DcMotor.class, "arm_length");
		hand = hardwareMap.get(Servo.class, "hand");
		finger = hardwareMap.get(Servo.class, "finger");
	}

	private void _initAprilTag() {
		AprilTagProcessor.Builder myAprilTagProcessorBuilder;
		VisionPortal.Builder myVisionPortalBuilder;
		VisionPortal myVisionPortal;

		myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
		myAprilTagProcessor = myAprilTagProcessorBuilder.build();
		myVisionPortalBuilder = new VisionPortal.Builder();
		if (USE_WEBCAM) {
			// Use a webcam.
			myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
		} else {
			// Use the device's back camera.
			myVisionPortalBuilder.setCamera(BuiltinCameraDirection.FRONT);
		}
		myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
		myVisionPortal = myVisionPortalBuilder.build();
		telemetry.addData("AprilTag 초기화 됨", "대기중...");
		telemetry.update();
	}

	/**
	 * 모터 관련 여러기능 초기화
	 */
	private void _initMotor() {
		// 모터 방향 초기화
		left1.setDirection(DcMotor.Direction.FORWARD);
		left2.setDirection(DcMotor.Direction.FORWARD);
		right1.setDirection(DcMotor.Direction.REVERSE);
		right2.setDirection(DcMotor.Direction.REVERSE);
		arm_angle.setDirection(DcMotor.Direction.REVERSE);
		arm_length.setDirection(DcMotor.Direction.REVERSE);
		// TargetPosition 오차 설정
		((DcMotorEx) left1).setTargetPositionTolerance(10);
		((DcMotorEx) left2).setTargetPositionTolerance(10);
		((DcMotorEx) right1).setTargetPositionTolerance(10);
		((DcMotorEx) right2).setTargetPositionTolerance(10);
		// 모터 고정
		left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		arm_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		arm_length.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		// 모드 초기화(바퀴)
		left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		// 모드 초기화(팔)
		arm_angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		arm_length.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		arm_angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		arm_length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		telemetry.addData("모터 초기화 됨", "대기중...");
		telemetry.update();
	}

	private void _initIMU() { //                                                                     로고 방향                                                 USB포트 방향
		imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
		imu.resetYaw();
		telemetry.addData("IMU 초기화 됨", "시작 대기중");
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

	private void resetPID() {
		// PID변수
		integralSum = 0;
		lastError = 0;
		timer.reset();
	}

	/**
	 * 모터 관련 여러기능 초기화
	 */
	private void resetMotor() {
		int EncoderAvg;

		// 모드 초기화(바퀴)
		left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		// 평균값
		EncoderAvg = (left1.getCurrentPosition() + left2.getCurrentPosition() + right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
	}

	private void callIMU() {
		YawPitchRollAngles orientation;
		AngularVelocity angularVelocity;

		orientation = imu.getRobotYawPitchRollAngles();
		angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
		yaw = orientation.getYaw(AngleUnit.DEGREES);
	}

	private void turnBot(double output) {
		left1.setPower(-(turnSpeed * output));
		left2.setPower(-(turnSpeed * output));
		right1.setPower(turnSpeed * output);
		right2.setPower(turnSpeed * output);
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

	private void mecanmTurnCCW(double targetAngle) {
		imu.resetYaw();
		resetPID();
		// rx = -(Math.abs(power) * (targetAngle / Math.abs(targetAngle)));
		while (Math.abs(angleWrap(targetAngle - yaw)) > 1) {
			callIMU();
			turnBot(PIDController(targetAngle, yaw));
		}
		left1.setPower(0);
		left2.setPower(0);
		right1.setPower(0);
		right2.setPower(0);
	}

	/**
	 * https://www.ctrlaltftc.com/the-pid-controller
	 */
	private double PIDController(double reference, double now) {
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

	private void setArmLength(int encoder) {
		arm_length.setTargetPosition(encoder);
		arm_length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	/**0 ~ 13425 (360도)
	 * 6712.5 (180도)
	 * 3356.25 (90도)
	 * . . .**/
	private void setArmAngle(int angle) {
		int encoder = armAngleRev * angle / 360;
		arm_angle.setTargetPosition(encoder);
		arm_angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void movefinger(boolean value) {
		if (value) { finger.setPosition(1); }   // true     먹기
		else { finger.setPosition(0); }         // false    뱉기 (퉤!)
	}

	/**값 조절 할 것!**/
	public void moveHand(boolean value) {
		if (value) { hand.setPosition(0.7); }     // true     돌리기
		else { hand.setPosition(0.388); }           // false    원점
	}

	/**
	 * double x = x_targetDist
	 * double y = y_targetDist
	 * targetAngle
	 * targetAngle값을 잘 지정하면 포물선으로 갈지도?
	 */
	private void Move3(double x, double y, double targetAngle) {
		double dist = Math.sqrt(x * x + y * y);
		double error = dist - left1.getCurrentPosition();
		//x = Math.abs(power) * x;
		//y = Math.abs(power) * y;
		rx = 0;
		denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

		// 인코더 계산을 위한 용도로 초기화
		resetMotor();
		resetPID(); /////////// 확인
		// 바퀴 4개의 인코더 값의 합 / 4 == 바퀴 4개 인코더 값의 평균
		// 평균 - 목표거리 == 오차
		// 오차의 절대값 > 허용오차(1인치)
		while (opModeIsActive() && Math.abs(error) >= 10) {
			callIMU();
			error = dist - left1.getCurrentPosition();

			rx = PIDController(targetAngle, yaw);
			denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

			left1.setPower(((((y + x) - left1.getCurrentPosition()) + rx) / denominator) + 0.2);
			left2.setPower(((((y - x) - left2.getCurrentPosition()) - rx) / denominator) + 0.2);
			right1.setPower(((((y - x) - right1.getCurrentPosition()) + rx) / denominator) + 0.2);
			right2.setPower(((((y + x) - right2.getCurrentPosition()) - rx) / denominator) + 0.2);

			telemetry.addData("left1", left1.getCurrentPosition());
			telemetry.update();
		}
		left1.setPower(0);
		left2.setPower(0);
		right1.setPower(0);
		right2.setPower(0);
	}

	private void alignWithAprilTag(AprilTagDetection tag) {
		if (this.aprilTag.id == tag.id) {
			Move3(tag.ftcPose.x, 0, 0);
			mecanmTurnCCW(tag.ftcPose.bearing);
		}
	}

	private void aprilEx() {
		// Get a list of AprilTag detections.
		myAprilTagDetections = myAprilTagProcessor.getDetections();
		// Iterate through list and call a function to display info for each recognized AprilTag.
		for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
			aprilTag = myAprilTagDetection_item;
			if (aprilTag.metadata != null) {
				telemetry.addLine("" + JavaUtil.formatNumber(aprilTag.ftcPose.yaw, 6, 1));
			}
		}
	}

	@Override
	public void runOpMode() {
		// 인치당 인코더
		double COUNTS_PER_MOTOR_REV = 537;
		double DRIVE_GEAR_REDUCTION = 1;
		double WHEEL_DIAMETER_INCHES = 3.7;
		COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
		armAngleRev = 13425;

		isParking = false;
		isBasket = false;
		// 카메라 사용여부
		USE_WEBCAM = true;
		power = 0.6;
		turnSpeed = 0.5;
		sleepTime = 100;

		// 초기화
		_initHardWareMap();
		_initPID();
		_initAprilTag();
		_initMotor();
		_initIMU();
		telemetry.update();
		waitForStart();
		if (opModeIsActive()) {
			// Put run blocks here.
			while (opModeIsActive()) { // 반복
				callIMU();



//				autonomus();
				arm_angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				arm_angle.setTargetPosition(-3000);
				arm_angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				sleep(5000);

				telemetry.update();
			}
		}
	}

	private void autonomus() {
		if (isBasket == false) {
			Move3(0, COUNTS_PER_INCH * 24, 0);
			sleep(sleepTime);
			mecanmTurnCCW(90); ///////////////////// 확인
			sleep(sleepTime);
		}

		myAprilTagDetections = myAprilTagProcessor.getDetections();
		for (AprilTagDetection item : myAprilTagDetections) {
			aprilTag = item;
			if (aprilTag.metadata != null) {

				if (isParking == false) {
					if (isBasket == false) {
						isBasket = true;

						scoring_basket(aprilTag);
						continue;
					}
					else {
						isParking = true;
						parking(aprilTag);
					}
				}
			}
		}
	}

	/**이 함수는 끝나면 11 or 14 태그를 바라본다**/
	private void scoring_basket(AprilTagDetection tag) {
		if (tag.id == 13 || tag.id == 16) {
			callIMU();


			Move3(0, COUNTS_PER_INCH * (tag.ftcPose.range - 24), 0);
			sleep(sleepTime);
			alignWithAprilTag(tag);
			mecanmTurnCCW(45);
			// 팔 움직이는 코드
			sleep(sleepTime);
			//
			setArmAngle(180);
			sleep(1000);
			moveHand(false);		// 수정하기
			sleep(500);
			arm_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			sleep(100);
			arm_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			movefinger(true);
			sleep(500);
			setArmAngle(10);
			sleep(2000);

			// ↖ : ←  ↓
			mecanmTurnCCW(5 + (45+90));
		}
	}

	/**이 함수는 끝나면 11 or 14 태그를 바라본다**/
	private void parking(AprilTagDetection tag) {
		if (tag.id == 11 || tag.id == 14) {
			callIMU();
//      Move2(0, 1, COUNTS_PER_INCH * (myAprilTagDetection.ftcPose.range - 20), myAprilTagDetection.ftcPose.bearing);
//      sleep(1000);
//      // Y값이 -인지 확인하기
//      Move2(1, 0, COUNTS_PER_INCH * 25, 0);
			// 테스트 해보고 거리조절
			Move3(0, COUNTS_PER_INCH * (tag.ftcPose.range - 10), tag.ftcPose.bearing);
			sleep(sleepTime);
			alignWithAprilTag(tag);
			sleep(sleepTime);
			// X값이 -인지 확인하기
			Move3(COUNTS_PER_INCH * 20, 0, 0);
		}
	}
}
