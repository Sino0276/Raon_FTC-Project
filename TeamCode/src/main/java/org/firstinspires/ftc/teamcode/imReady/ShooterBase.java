package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterBase extends ShooterHardware {
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(2, 0.3, 0.001, 11);
    public static int TPS = 1000;
//    public static double SERVO_MAX = 1;
//    public static double SERVO_MIN = 0;

    public boolean  isShooterSpin = false,
                    isServoSpin = false;

    public ShooterBase(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void setVelocity(int TPS) {
        shooter1.setVelocity(TPS);
        shooter2.setVelocity(TPS);
    }

    public void setPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void setTPS(int TPS) {
        if (TPS > 2800) { ShooterBase.TPS = 2800; }
        else if (TPS < 0) { ShooterBase.TPS = 0; }
        else { ShooterBase.TPS = TPS; }
    }
    public void addTPS(int amount) { setTPS(TPS + amount); }

    // servo

    public void servoSpin(double power) {
        servo.setPower(power);
    }
}
