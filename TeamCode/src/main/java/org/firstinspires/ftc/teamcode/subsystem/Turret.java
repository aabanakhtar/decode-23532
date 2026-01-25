package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.utilities.BasicFilter;
import org.firstinspires.ftc.teamcode.utilities.IdentityFilter;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;

@Config
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();

    public static double PREDICT_FACTOR = -0.00; // -0.01

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
    public static double targetPower = 0.0;
    public static final double targetAngle = 0.0;

    // turret gains
    public static double kP = 0.04;
    public static double kI = 0.0;
    // was 0.001
    public static double kD = 0.000;

    public static int SETPOINT_SMOOTHING_WINDOW_RANGE = 4;
    private final BasicFilter setpointFilter = new RunningAverageFilter(SETPOINT_SMOOTHING_WINDOW_RANGE);

    public static int ABS_ANGLE_WINDOW_RANGE = 4;
    private final BasicFilter analogEncoderFilter = new RunningAverageFilter(ABS_ANGLE_WINDOW_RANGE);

    // PIDs
    private final Controller turretAnglePID = new PIDFController(kP, kI, kD, 0);
    // we have two for different sizes of error

    // 312 RPM Yellow Jacket with gearing 27t to 95t
    public static final double GEAR_RATIO = 95.0 / 27.0; // motor rotations per turret rotation
    public static final double TURRET_ENCODER_CPR = 537.7 * GEAR_RATIO; // ≈ 1891.6 ticks per turret rotation
    // limits the turret's use of abs encoder beyond this area
    public static final double MAX_SAFE_ANGLE = 150.0; //deg
    public static double MAX_ANGULAR_HOME_VELOCITY = 5.0; // deg/s
    public static double MAX_DISPARITY = 10.0; // deg
    public static final double TURRET_MAX_ANGLE = 180.0; // deg
    public static final double TURRET_PID_TOLERANCE = 1.0; //deg

    // for relocalizing turret
    public static double TURRET_HOME_OFFSET = 0;

    public Turret() {
        turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
    }

    @Override
    public void periodic() {
        // always use quadrature
        final double encoderAngle = calculateAngleFromEncoder();
        analogEncoderFilter.updateValue(robot.analogEncoder.getCurrentPosition());

        robot.flightRecorder.addLine("==========TURRET===========");
        robot.flightRecorder.addData("angle", encoderAngle);
        robot.flightRecorder.addData("target angle", setpointFilter.getFilteredOutput());

        if (tuning) {
            ((PIDFController)turretAnglePID).setPIDF(kP, kI, kD, 0);
        }

        // update the queued mode
        mode = pendingMode;
        switch (mode) {
            case RAW:
                robot.shooterTurret.set(targetPower);
                break;

            // resetting encoder
            case HOMING: {
                TURRET_HOME_OFFSET = robot.analogEncoder.getCurrentPosition();
                pendingMode = Mode.PINPOINT;
                robot.shooterTurret.stopAndResetEncoder();
                break;
            }

            // OPTION A: use pinpoint to aim turret
            case PINPOINT: {
                if (lastMode != Mode.PINPOINT) {
                    turretAnglePID.reset();
                    setpointFilter.reset();
                }

                attemptRelocalize();

                double predictedLeadOffset = Math.toDegrees(robot.drive.getTangentVelocityToGoal() * PREDICT_FACTOR);

                // filter our target
                double rawTarget = robot.drive.getAimTarget().heading;
                setpointFilter.updateValue(rawTarget);

                double filteredTarget = setpointFilter.getFilteredOutput() - predictedLeadOffset;

                // constrain our angles
                double constrainedAngleDeg = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, filteredTarget));
                double power = turretAnglePID.calculate(encoderAngle, constrainedAngleDeg);

                // absolute limit (idk if this helps but should)
                if (isAtTarget() || (power > 0 && encoderAngle > TURRET_MAX_ANGLE) || (power < 0 && encoderAngle < -TURRET_MAX_ANGLE)) {
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

    private void attemptRelocalize() {
        // max speed angular and axis
        if (Math.abs((robot.shooterTurret.getCorrectedVelocity() / TURRET_ENCODER_CPR) * 360.0) > MAX_ANGULAR_HOME_VELOCITY) {
            return;
        }
        // are we shooting (vibration)
        if (robot.shooter.getMode() == Shooter.Mode.DYNAMIC) {
            return;
        }
        // voltage
        if (robot.getVoltage() < DuneStrider.IDEAL_VOLTAGE) {
            return;
        }
        // is encoder at max? Don't trust sign flipping here
        if (Math.abs(calculateAngleFromEncoder()) >= MAX_SAFE_ANGLE) {
            return;
        }

        // home
        double error = analogEncoderFilter.getFilteredOutput() - calculateAngleFromEncoder();
        if (error > MAX_DISPARITY) {
            return;
        }
        TURRET_HOME_OFFSET += AngleUnit.normalizeDegrees(error);
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

    private double calculateAngleFromEncoder() {
        // results in 90 <---- 0 -----> -90
        double encoderPos = robot.shooterTurret.encoder.getPosition();
        return TURRET_HOME_OFFSET + (encoderPos / TURRET_ENCODER_CPR) * 360.0;
    }
}
