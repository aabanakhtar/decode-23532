package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();

    public enum Mode {
        HOMING,
        AIMING,
        RAW
    }

    public static Mode mode = Mode.RAW;

    public static boolean tuning = false;
    public static double targetPower = 0.0;
    public static double targetAngle = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // 312 RPM Yellow Jacket with gearing 27t to 95t
    public static final double GEAR_RATIO = 95.0 / 27.0; // motor rotations per turret rotation
    public static final double TURRET_ENCODER_CPR = 537.7 * GEAR_RATIO; // ≈ 1891.6 ticks per turret rotation
    public static final double TURRET_MAX_ANGLE = 84.56;
    public static final double TURRET_HOME_OFFSET = TURRET_ENCODER_CPR * (TURRET_MAX_ANGLE / 360.0); // to set right to negative

    public static double homingPower = 0.0;
    public static PIDFController controller = new PIDFController(kP, kI, kD, 0);

    public Turret() {

    }

    @Override
    public void periodic() {
        robot.telemetry.addLine("==========TURRET===========");
        robot.telemetry.addData("Encoder position", robot.shooterTurret.encoder.getPosition());
        robot.telemetry.addData("angle", calculateAngleFromEncoder());
        robot.telemetry.addData("target angle", targetAngle);
        robot.telemetry.addData("target power:", targetPower);

        switch (mode) {
            case RAW:
                robot.shooterTurret.set(targetPower);
                break;
            case AIMING:
                // aim within hardware limits
                double targetAngleFixed = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, targetAngle));
                double power = controller.calculate(calculateAngleFromEncoder(), targetAngleFixed);
                robot.shooterTurret.set(power);
                break;
            case HOMING:
                robot.shooterTurret.set(homingPower);
                if (isAtHome()) {
                    mode = Mode.AIMING;
                    robot.shooterTurret.stopAndResetEncoder();
                }
                break;
        }
    }

    public void setMode(Mode amode) {
        mode = amode;
    }

    public boolean isAtHome() {
        return robot.shooterTurret.isOverCurrent();
    }

    private double calculateAngleFromEncoder() {
        // right = negative in standard mathematics ( i think )
        double encoderPos = TURRET_HOME_OFFSET + robot.shooterTurret.encoder.getPosition();
        double angle = (encoderPos / TURRET_ENCODER_CPR) * 360.0;
        angle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
        return angle;
    }
}
