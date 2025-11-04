package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoTest3")
public class AutoTest3 extends LinearOpMode {
    public class Shoot {
        private DcMotorEx shooterLeft, shooterRight;
        public PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(2, 0.3, 0.001, 11);


        public Shoot(HardwareMap hardwareMap) {
            shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
            shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
            shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

            motorSetup(shooterLeft, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
            motorSetup(shooterRight, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);

            shooterLeft.setVelocityPIDFCoefficients(SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f);
            shooterRight.setVelocityPIDFCoefficients(SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f);
        }

        private void motorSetup(DcMotorEx motor,
                                DcMotorEx.Direction direction,
                                DcMotorEx.RunMode runMode,
                                DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
            motor.setDirection(direction);
            motor.setMode(runMode);
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }

        public class Shooting implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shooterLeft.setVelocity(1000);
                    shooterRight.setVelocity(1000);
                    initialized = true;
                }

                double vel = shooterLeft.getVelocity();
                packet.put("shooter vel", vel);
                if (abs(1000 - vel) > 10) { // 오차가 +- 50 이내 이면 탈출
                    return true;
                } else {

                    return false;
                }
            }
        }
        public Action shooting() {
            return new Shooting();
        }

        public class UnShooting implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shooterLeft.setVelocity(0);
                    shooterRight.setVelocity(0);
                    initialized = true;
                }

                return false;
            }
        }
        public Action unshooting(){
            return new UnShooting();
        }
    }

    public class Push {
//        private CRServo pusher;
        private Servo pusher;

        public Push(HardwareMap hardwareMap) {
//            pusher = hardwareMap.get(CRServo.class, "servo");
            pusher = hardwareMap.get(Servo.class, "servo");
        }

        public class SpinPusher implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pusher.setPosition(0);
                    initialized = true;
                }
//                pusher.setPower(1);

                if (pusher.getPosition() == 0) {
                    return false;
                } else {
                    return true;
                }
//                return false;
            }
        }
        public Action spinPusher() {
            return new SpinPusher();
        }

        public class UnSpinPusher implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pusher.setPosition(1);
                    initialized = true;
                }
//                pusher.setPower(1);

                if (pusher.getPosition() == 1) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action unSpinPusher() {
            return new UnSpinPusher();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shoot shoot = new Shoot(hardwareMap);
        Push push = new Push(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .afterDisp(1, shoot.unshooting())  /// ////////////////// 이거 테스트
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(push.unSpinPusher());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        shoot.shooting(),
                        push.spinPusher(),
//                        push.unSpinPusher(),
//                        shoot.unshooting(),
                        trajectoryActionCloseOut
                )
        );
    }
}
