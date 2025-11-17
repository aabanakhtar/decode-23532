package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();


    public enum Mode {
        HOMING,
        RAW,
        DYNAMIC,
        FIXED
    }

    public static Mode mode = Mode.RAW;

    public static boolean tuning = false;
    public static double targetPower = 0.0;
    public static final double targetAngle = 0.0;
    public static double kP = 0.015;
    public static double kI = 0.0;
    public static double kD = 0.003;
    public static double tagKP = 0.0;
    public static double tagKD = 0.0;

    // 1620 RPM Yellow Jacket with gearing 27t to 95t
    public static final double GEAR_RATIO = 95.0 / 27.0; // motor rotations per turret rotation
    public static final double TURRET_ENCODER_CPR = 103.8 * GEAR_RATIO; // ≈ 1891.6 ticks per turret rotation
    public static final double TURRET_MAX_ANGLE = 74.56;
    public static final double TURRET_PID_TOLERANCE = 3.0;
    public static final double TURRET_HOME_OFFSET = -TURRET_ENCODER_CPR * (84.56 / 360.0); // to set right to negative
    public static double homingPower = -0.35;
    public static PIDFController turretAnglePID = new PIDFController(kP, kI, kD, 0);

    public Turret() {
        turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
    }

    @Override
    public void periodic() {
        robot.telemetry.addLine("==========TURRET===========");
        robot.telemetry.addData("Encoder position", robot.shooterTurret.encoder.getPosition());
        robot.telemetry.addData("angle", calculateAngleFromEncoder());
        robot.telemetry.addData("target angle", targetAngle);
        robot.telemetry.addData("target power:", targetPower);
        robot.telemetry.addData("is at home:", isAtHome());

        if (tuning) {
            turretAnglePID.setPIDF(kP, 0, kD, 0);
        }

        switch (mode) {
            case RAW:
                robot.shooterTurret.set(targetPower);
                break;

            case HOMING: {
                robot.shooterTurret.set(homingPower);
                if (isAtHome()) {
                    mode = Mode.DYNAMIC;
                    robot.shooterTurret.stopAndResetEncoder();
                }
                break;
            }
            case DYNAMIC: {
                double target = robot.drive.getAimTarget().heading;
                double power = turretAnglePID.calculate(calculateAngleFromEncoder(), target);
                robot.shooterTurret.set(power);
                break;
            }
            case FIXED: {
                double power = turretAnglePID.calculate(calculateAngleFromEncoder(), 0);
                robot.shooterTurret.set(power);
                break;
            }
        }
    }


    public void setMode(Mode amode) {
        mode = amode;
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isAtHome() {
        // TODO: tune
        return robot.shooterTurret.motorEx.getCurrent(CurrentUnit.AMPS) > 100000.0;
    }

    public boolean isAtTarget() {
        return turretAnglePID.atSetPoint();
    }

    private double calculateAngleFromEncoder() {
        // results in 90 <---- 0 -----> -90
        double encoderPos = TURRET_HOME_OFFSET + robot.shooterTurret.encoder.getPosition();
        double angle = (encoderPos / TURRET_ENCODER_CPR) * 360.0;
        angle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
        return angle;
    }
}
