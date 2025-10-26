package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.Arrays;
import java.util.List;

public abstract class MecanumHardware {
    protected DcMotorEx mtr_rr, mtr_rf, mtr_lr, mtr_lf;
    protected IMU imu;
//    protected GoBildaPinpointDriver pinpoint;

    public MecanumHardware(HardwareMap hardwareMap) {
        mtr_rr = hardwareMap.get(DcMotorEx.class, "rightRear");
        mtr_rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        mtr_lr = hardwareMap.get(DcMotorEx.class, "leftRear");
        mtr_lf = hardwareMap.get(DcMotorEx.class, "leftFront");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        initMotor();
//        initPinpoint();
    }

//    private void initPinpoint() {
//        pinpoint.initialize();
//        pinpoint.setOffsets(3, -72, DistanceUnit.MM);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//    }
    private void initMotor() {
        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        motorSetup(mtr_rr, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(mtr_rf, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(mtr_lr, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
        motorSetup(mtr_lf, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
    }

    private void motorSetup(DcMotorEx motor,
                              DcMotorEx.Direction direction,
                              DcMotorEx.RunMode runMode,
                              DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

}
