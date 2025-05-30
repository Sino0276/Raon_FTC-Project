package org.firstinspires.ftc.teamcode.mecanum.manipulator;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmAngleManipulator extends ArmManipulator {
    private double gear_ratio;
      // 모델에 따라 바꿔야 함
    private double tics_per_degree;

    private double kp = 0.1;

    public ArmAngleManipulator(DcMotor.Direction direction, float gear_ratio) {
        super(direction);
        this.gear_ratio = gear_ratio;

        encoder_resolution *= gear_ratio;   // 한바퀴 도는데 필요한 인코더 값
        tics_per_degree = encoder_resolution / 360; // encoder per degree
    }

    private double forward_tics() {
        return (arm_angle.getCurrentPosition());
    }



    // 필히 수정
    public void move(float degree, double max_speed) {
        reset_encoders();

        max_speed = abs(max_speed);     // 무조건 양수로
        double forward_direction_mult = (degree > 0.0) ? 1.0 : -1.0;
        double forward_target_tics = tics_per_degree * degree /**
                forward_direction_mult*/;

        while (true) {
            double current_forward_tics = /*forward_direction_mult **/ forward_tics();
            double error = forward_target_tics - current_forward_tics;
            linearOpMode.telemetry.addLine().addData("error", error);

            if (abs(error) <= abs(tollerance)) {
                break;
            }
            // double error = expected_heading - heading;
            double speed_mult = power_accel_decel(current_forward_tics,
                    forward_target_tics, mtr_accel_min,
                    mtr_decel_min, mtr_accel_tics, mtr_decel_tics);
            double speed = speed_mult * error / 1000;
            linearOpMode.telemetry.addLine().addData("speed", speed);
//            speed /= (abs(speed) > 1) ? speed : 1;
            setSpeeds(speed * max_speed);
            linearOpMode.telemetry.update();
        }
        setSpeeds(0);
    }
}
