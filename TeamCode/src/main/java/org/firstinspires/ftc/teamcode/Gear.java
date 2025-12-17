package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name = "Gear")
@Disabled
public class Gear extends OpMode {

    public static double TPS = 0;
    public static boolean MOTOR_ON = false;
    public static PIDFCoefficients MOTOR_PID = new PIDFCoefficients(0, 0, 0, 0);

    private final double PPR = 145.1; // Pulses Per Revolution

    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        motor.setVelocityPIDFCoefficients(MOTOR_PID.p, MOTOR_PID.i, MOTOR_PID.d, MOTOR_PID.f);

        if (MOTOR_ON) {
//            motor.setVelocity(TPS);
            motor.setPower(TPS);
        } else {
            motor.setPower(0);
        }

//        packet.put("Target RPM", RPM);
//        packet.put("Motor RPM", ticksPerSecondToRpm(motor.getVelocity()));
        packet.put("Target Velocity (TPS)", TPS);
        packet.put("Motor Velocity (TPS)", motor.getVelocity());
        dashboard.sendTelemetryPacket(packet);
    }

//    private double rpmToTicksPerSecond(double rpm) {
//        return rpm * (PPR / 60.0);
//    }
//
//    private double ticksPerSecondToRpm(double ticksPerSecond) {
//        return (ticksPerSecond * 60.0) / PPR;
//    }
}
