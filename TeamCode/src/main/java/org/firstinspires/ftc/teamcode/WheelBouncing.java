package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "WheelBouncing")
public class WheelBouncing extends OpMode {
    public static DcMotorSimple.Direction LEFT_SHOOTER_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;
    public static double RPM = 0;
    private double maxRPM = 1150;
    private final String LEFT_SHOOTER_MOTOR = "leftShooter",
                         RIGHT_SHOOTER_MOTOR = "rightShooter";

    private DcMotorEx leftShooter, rightShooter;

    FtcDashboard dashboard;
    MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER_MOTOR);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER_MOTOR);

        DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        motorSetup(leftShooter, LEFT_SHOOTER_DIR, mode);
        motorSetup(rightShooter, RIGHT_SHOOTER_DIR, mode);

        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        motorControl();
        updateTelemetry();
    }

    private void motorSetup(DcMotorEx motor, DcMotorSimple.Direction direction, DcMotor.RunMode mode) {
        if (motor == null || direction == null || mode == null) {
            throw new IllegalArgumentException("Motor, Direction and Mode cannot be null");
        }

        motor.setDirection(direction);
        motor.setMode(mode);
    }

    private void setRPM(double rpm) {
        double power = rpm / maxRPM;
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    private void motorControl() {
        RPM = Range.clip(RPM, 0, maxRPM);
        setRPM(RPM);
    }

    private void updateTelemetry() {
        multipleTelemetry.addData("MaxRPM", maxRPM);
        multipleTelemetry.addData("TargetRPM", RPM);
        multipleTelemetry.update();
    }

}
