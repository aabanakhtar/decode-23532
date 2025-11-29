package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();

    public enum Mode {
        HOMING,
        RAW,
        PINPOINT,
        DEBUG,
        LIMELIGHT
    }

    // State machine related things
    public static Mode mode = Mode.RAW;
    public static Mode pendingMode = Mode.RAW;
    public static Mode lastMode = Mode.RAW;

    public static boolean tuning = false;
    public static double targetPower = 0.0;
    public static final double targetAngle = 0.0;

    // turret gains
    public static double kP = 0.04;
    public static double kI = 0.0;
    public static double kD = 0.001;

    // limelight gains
    public static double LIMELIGHT_FAR_kP = -0.013;
    public static double LIMELIGHT_FAR_kD = 0.0;

    public static double LIMELIGHT_CLOSE_kP = -0.03;
    public static double LIMELIGHT_CLOSE_kD = 0;

    public static double LIMELIGHT_SMALL_GAINS_THRESHOLD = 11.0;  /* degrees */
    public static double LIMELIGHT_TRANSITION_GAINS_THRESHOLD = 20.0;
    public static double LIMELIGHT_LOWER_TRANSITION_GAINS_THRESHOLD = 10.0;

    public static InterpLUT limelightPGains = new InterpLUT();

    static {
        limelightPGains.add(0, LIMELIGHT_CLOSE_kP);
        limelightPGains.add(LIMELIGHT_LOWER_TRANSITION_GAINS_THRESHOLD, LIMELIGHT_CLOSE_kP);
        limelightPGains.add(LIMELIGHT_TRANSITION_GAINS_THRESHOLD, LIMELIGHT_FAR_kP);
        limelightPGains.add(180, LIMELIGHT_FAR_kP);
    }

    // PIDs
    public static PIDFController turretAnglePID = new PIDFController(kP, kI, kD, 0);
    // we have two for different sizes of error
    public static PIDFController limelightLargeErrorPID = new PIDFController(LIMELIGHT_FAR_kP, 0, LIMELIGHT_FAR_kD, 0);
    public static PIDFController limelightSmallErrorPID = new PIDFController(LIMELIGHT_CLOSE_kP, 0, LIMELIGHT_CLOSE_kD, 0);

    // 1620 RPM Yellow Jacket with gearing 27t to 95t
    public static final double GEAR_RATIO = 95.0 / 27.0; // motor rotations per turret rotation
    public static final double TURRET_ENCODER_CPR = 103.8 * GEAR_RATIO; // ≈ 1891.6 ticks per turret rotation
    public static final double TURRET_MAX_ANGLE = 74.56;
    public static final double TURRET_PID_TOLERANCE = 1.0;
    public static final double TURRET_HOME_OFFSET = -TURRET_ENCODER_CPR * (84.56 / 360.0); // to set right to negative
    public static final double HOME_POWER = -0.35;

    public Turret() {
        turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
    }

    @Override
    public void periodic() {
        robot.flightRecorder.addLine("==========TURRET===========");
        robot.flightRecorder.addData("Mode", mode.toString());
        robot.flightRecorder.addData("Encoder position", robot.shooterTurret.encoder.getPosition());
        robot.flightRecorder.addData("angle", calculateAngleFromEncoder());
        robot.flightRecorder.addData("target angle", targetAngle);
        robot.flightRecorder.addData("target power:", targetPower);
        robot.flightRecorder.addData("is at home:", isAtHome());

        if (tuning) {
            turretAnglePID.setPIDF(kP, 0, kD, 0);
            limelightLargeErrorPID.setPIDF(LIMELIGHT_FAR_kP, 0, LIMELIGHT_FAR_kD, 0);
            limelightSmallErrorPID.setPIDF(LIMELIGHT_CLOSE_kP, 0, LIMELIGHT_CLOSE_kD, 0);
        }

        Double lastTx = robot.eyes.getLastTx();
        if (lastTx != null) {
            robot.flightRecorder.addData("adjusted tx", adjustTx(lastTx));
        }

        // update the queued mode
        mode = pendingMode;
        switch (mode) {
            case RAW:
                robot.shooterTurret.set(targetPower);
                break;

            // resetting encoder
            case HOMING: {
                robot.shooterTurret.set(HOME_POWER);

                if (isAtHome()) {
                    pendingMode = Mode.PINPOINT;
                    robot.shooterTurret.stopAndResetEncoder();
                }
                break;
            }

            // OPTION A: use pinpoint to aim turret
            case PINPOINT: {
                if (lastMode != Mode.PINPOINT) {
                    turretAnglePID.reset();
                }

                if (robot.eyes.getLastTx() != null) {
                    pendingMode = Mode.LIMELIGHT;
                    break;
                }

                double target = robot.drive.getAimTarget().heading;
                double power = turretAnglePID.calculate(calculateAngleFromEncoder(), target);
                robot.shooterTurret.set(power);
                break;
            }

            // Option B: Use limelight smart cam to aim turret
            case LIMELIGHT: {
                Double tx = robot.eyes.getLastTx();
                if (tx == null) {
                    pendingMode = Mode.PINPOINT;
                    break;
                }

                if (lastMode != Mode.LIMELIGHT) {
                    limelightLargeErrorPID.reset();
                    limelightSmallErrorPID.reset();
                }

                // use gain scheduling to make the controller better
                double error = adjustTx(tx);
                PIDFController scheduledController =
                        Math.abs(error) > LIMELIGHT_SMALL_GAINS_THRESHOLD ?
                                limelightLargeErrorPID : limelightSmallErrorPID;
                double power = scheduledController.calculate(error, 0);
                double interpolatedPGain = limelightPGains.get(error);
                //double power = interpolatedPGain * error;

                robot.shooterTurret.set(power);
                break;
            }

            // Debug purposes only
            case DEBUG: {
                double power = turretAnglePID.calculate(calculateAngleFromEncoder(), 0);
                robot.shooterTurret.set(power);
                break;
            }
        }

        // store this for smooth transitions between the two (rising-edge)
        lastMode = mode;
    }

    public void setMode(Mode amode) {
        pendingMode = amode;
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isAtHome() {
        return robot.shooterTurret.motorEx.getCurrent(CurrentUnit.AMPS) > 100000000;
    }

    public boolean isAtTarget() {
        return turretAnglePID.atSetPoint();
    }

    private double adjustTx(double tx) {
        tx = -tx;
        double current = calculateAngleFromEncoder();
        double desired = current + tx;

        // clamp
        if (desired > TURRET_MAX_ANGLE) {
            return TURRET_MAX_ANGLE - current;
        }
        if (desired < -TURRET_MAX_ANGLE) {
            return -TURRET_MAX_ANGLE - current;
        }

        return tx;
    }

    private double calculateAngleFromEncoder() {
        // results in 90 <---- 0 -----> -90
        double encoderPos = TURRET_HOME_OFFSET + robot.shooterTurret.encoder.getPosition();
        double angle = (encoderPos / TURRET_ENCODER_CPR) * 360.0;
        angle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
        return angle;
    }
}
