package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.utilities.BasicFilter;
import org.firstinspires.ftc.teamcode.utilities.IdentityFilter;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;

@Configurable
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();

    public enum Mode {
        HOMING,
        RAW,
        PINPOINT,
        DEBUG,
    }

    // State machine related things
    public static Mode mode = Mode.RAW;
    public static Mode pendingMode = Mode.RAW;
    public static Mode lastMode = Mode.RAW;

    public static boolean tuning = false;
    public static double autonomousEncoderOffset = 0.0;
    public static double targetPower = 0.0;
    public static final double targetAngle = 0.0;

    // turret gains
    public static double kP = 0.075;
    public static double kI = 0.0;
    public static double kD = 0.001;

    public static int SETPOINT_SMOOTHING_WINDOW_RANGE = 6;
    public BasicFilter setpointFilter = new RunningAverageFilter(SETPOINT_SMOOTHING_WINDOW_RANGE);

    // limelight gains
    public static double LIMELIGHT_FAR_kP = 0.0;
    public static double LIMELIGHT_FAR_kD = 0.0;

    public static double LIMELIGHT_CLOSE_kP = 0.0;
    public static double LIMELIGHT_CLOSE_kD = 0;

    public static double LIMELIGHT_SMALL_GAINS_THRESHOLD = 11.0;  /* degrees */
    public static double LIMELIGHT_TRANSITION_GAINS_THRESHOLD = 20.0;
    public static double LIMELIGHT_LOWER_TRANSITION_GAINS_THRESHOLD = 10.0;


    // PIDs
    public static Controller turretAnglePID = new PIDFController(kP, kI, kD, 0);
    // we have two for different sizes of error
    public static PIDFController limelightLargeErrorPID = new PIDFController(LIMELIGHT_FAR_kP, 0, LIMELIGHT_FAR_kD, 0);
    public static PIDFController limelightSmallErrorPID = new PIDFController(LIMELIGHT_CLOSE_kP, 0, LIMELIGHT_CLOSE_kD, 0);

    // 312 RPM Yellow Jacket with gearing 27t to 95t
    public static final double GEAR_RATIO = 95.0 / 27.0; // motor rotations per turret rotation
    public static final double TURRET_ENCODER_CPR = 537.7 * GEAR_RATIO; // ≈ 1891.6 ticks per turret rotation
    public static final double TURRET_MAX_ANGLE = 74.56;
    public static final double TURRET_PID_TOLERANCE = 1.0;
    public static final double TURRET_HOME_OFFSET = -TURRET_ENCODER_CPR * (84.56 / 360.0); // to set right to negative
    public static final double HOME_POWER = -0.3;

    public Turret() {
        turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
    }

    @Override
    public void periodic() {
        final double encoderAngle = calculateAngleFromEncoder();
        robot.flightRecorder.addLine("==========TURRET===========");
        robot.flightRecorder.addData("Mode", mode.toString());
        robot.flightRecorder.addData("Encoder position", robot.shooterTurret.encoder.getPosition());
        robot.flightRecorder.addData("angle", encoderAngle);
        robot.flightRecorder.addData("target angle", targetAngle);
        robot.flightRecorder.addData("target power:", targetPower);
        robot.flightRecorder.addData("is at home:", isAtHome());

        if (tuning) {
            ((PIDFController)turretAnglePID).setPIDF(kP, kI, kD, 0);
            limelightLargeErrorPID.setPIDF(LIMELIGHT_FAR_kP, 0, LIMELIGHT_FAR_kD, 0);
            limelightSmallErrorPID.setPIDF(LIMELIGHT_CLOSE_kP, 0, LIMELIGHT_CLOSE_kD, 0);
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
                    setpointFilter.reset();
                }

                double rawTarget = robot.drive.getAimTarget().heading;
                setpointFilter.updateValue(rawTarget);

                double filteredTarget = setpointFilter.getFilteredOutput();
                double power = turretAnglePID.calculate(encoderAngle, filteredTarget);

                robot.flightRecorder.addData("pinpoint target", filteredTarget);
                if (isAtTarget()) {
                    robot.shooterTurret.set(0.0);
                    break;
                }

                robot.shooterTurret.set(power);
                break;
            }
            // Debug purposes only
            case DEBUG: {
                double power = turretAnglePID.calculate(encoderAngle, 0);
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
        return false;
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
