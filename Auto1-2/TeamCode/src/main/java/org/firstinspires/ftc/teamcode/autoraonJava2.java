package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "autoraonJava2")
public class autoraonJava2 extends LinearOpMode{
	//MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

	// public? private?
	// HardwareSystem이라는 Manager급 클래스 (고려)
	private DcMotor mtr_lf;
	private DcMotor mtr_lr;
	private DcMotor mtr_rf;
	private DcMotor mtr_rr;
	private DcMotor arm_angle;
	private DcMotor arm_length;
	//private Servo finger;
	private Servo hand;
	public IMU imu_0;

	public List<AprilTagDetection> myAprilTagDetections;
	public AprilTagDetection aprilTag;
	public AprilTagProcessor myAprilTagProcessor;

	boolean USE_WEBCAM;

	private int sleepTime;

	private void _init() {
		_initHardWareMap();
		//_initAprilTag();
		telemetry.update();

		mecanumCtrl = new MecanumController(this, mtr_lf, mtr_lr, mtr_rf, mtr_rr);
		armCtrl = new ArmController(this, arm_length, arm_angle);
		handCtrl = new HandController(this, hand);
		PIDCtrl = new PIDController(this);
		IMUCtrl = new IMUController(this);
		//apriltagCtrl = new AprilTagController(this);

		//auto_parking = new Parking(this);

	}

	private void _initHardWareMap() {
		mtr_lf = hardwareMap.get(DcMotor.class, "mtr_lf");
		mtr_lr = hardwareMap.get(DcMotor.class, "mtr_lr");
		mtr_rf = hardwareMap.get(DcMotor.class, "mtr_rf");
		mtr_rr = hardwareMap.get(DcMotor.class, "mtr_rr");
		imu_0 = hardwareMap.get(IMU.class, "imu_0");
		arm_angle = hardwareMap.get(DcMotor.class, "arm_angle");
		arm_length = hardwareMap.get(DcMotor.class, "arm_length");
		hand = hardwareMap.get(Servo.class, "hand");
	}

	MecanumController mecanumCtrl;
	ArmController armCtrl;
	HandController handCtrl;
	PIDController PIDCtrl;
	IMUController IMUCtrl;
	//AprilTagController apriltagCtrl;

	//Parking auto_parking;

	@Override
	public void runOpMode() {
		// 카메라 사용 여부
		USE_WEBCAM = true;
		// 초기화
		_init();
		telemetry.update();

		waitForStart();
		if (opModeIsActive()) {
//			while (opModeIsActive()) { // 반복
////				 aprilTag = apriltagCtrl.getAprilTag();
////
//				mecanumCtrl.mecanumMove(Utilities.inchToEncoder(24*2), 0, 0);
//
//				telemetry.update();
//			}
			sleep(1000);
			mecanumCtrl.mecanumMove(Utilities.inchToEncoder(24*2), 0, 0);
		}
	}
}
