package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();


    public enum Mode {
        HOMING,
        AIMING,
        RAW,
        FIXED
    }

    public static Mode mode = Mode.RAW;

    public static boolean tuning = false;
    public static double targetPower = 0.0;
    public static double targetAngle = 0.0;
    public static double kP = 0.009;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // 1620 RPM Yellow Jacket with gearing 27t to 95t
    public static final double GEAR_RATIO = 95.0 / 27.0; // motor rotations per turret rotation
    public static final double TURRET_ENCODER_CPR = 103.8 * GEAR_RATIO; // ≈ 1891.6 ticks per turret rotation
    public static final double TURRET_MAX_ANGLE = 84.56;
    public static final double TURRET_PID_TOLERANCE = 4.0;
    public static final double TURRET_HOME_OFFSET = -TURRET_ENCODER_CPR * (TURRET_MAX_ANGLE / 360.0); // to set right to negative

    public static double homingPower = -0.2;
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
            case AIMING: {
                // aim within hardware limits
                double encoderAngle = calculateAngleFromEncoder();
                // the location of the goal
                double goalHeading = robot.drive.getShooterPositionPinpointRel().heading;
                double robotHeading = Math.toDegrees(robot.drive.getPose().getHeading());
                // the location of the turret with respect to robot and field
                // turret heading is a range between around +- 85 degrees and is facing out the back of the bot, hence the 180 degrees
                double turretHeading = robotHeading + 180.0 + encoderAngle;
                double targetChange = normalizeAngle(goalHeading - turretHeading);
                double target = encoderAngle + targetChange;
                // clip the range to a suitable maximum (hard stops, maximize travel while we can)
                double targetAngleFixed = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, target));
                double power = turretAnglePID.calculate(calculateAngleFromEncoder(), targetAngleFixed);
                robot.shooterTurret.set(power);
                break;
            }
            case HOMING: {
                robot.shooterTurret.set(homingPower);
                if (isAtHome()) {
                    mode = Mode.FIXED;
                    targetAngle = 0;
                    robot.shooterTurret.stopAndResetEncoder();
                }
                break;
            }
            case FIXED: {
                double targetAngleFixed = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, targetAngle));
                double power = turretAnglePID.calculate(calculateAngleFromEncoder(), targetAngleFixed);
                robot.shooterTurret.set(power);
                break;
            }
        }
    }

    public void setMode(Mode amode) {
        mode = amode;
    }

    public void setTarget(double angle) {
        mode = Mode.AIMING;
        targetAngle = angle;
    }

    public boolean isAtHome() {
        return robot.shooterTurret.motorEx.isOverCurrent();
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

    private double normalizeAngle(double angleDegrees) {
        double angle_rad = Math.toRadians(angleDegrees);
        while (angle_rad > Math.PI) {
            angle_rad -= 2 * Math.PI;
        }
        while (angle_rad < -Math.PI) {
            angle_rad += 2 * Math.PI;
        }
        return Math.toDegrees(angle_rad);
    }
}
