package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.utilities.BasicFilter;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;

@Config
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();

    public static double PREDICT_FACTOR = -0.00;

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
    public static double targetAngle = 0.0;
    private static double lastAngle = 0.0;

    public static double TURRET_APPROACH_kP = 0.02;
    public static  double TURRET_PID_SWITCH = 3.0;

    // turret gains
    public static double kP = 0.03;
    public static double kI = 0.0;
    // was 0.001
    public static double kD = 0.000;

    public static int SETPOINT_SMOOTHING_WINDOW_RANGE = 4;
    private final BasicFilter setpointFilter = new RunningAverageFilter(SETPOINT_SMOOTHING_WINDOW_RANGE);

    // PIDs
    private final Controller turretAnglePID = new PIDFController(kP, kI, kD, 0);
    // we have two for different sizes of error

    // 312 RPM Yellow Jacket with gearing 27t to 95t
    public static final double GEAR_RATIO = 95.0 / 27.0; // motor rotations per turret rotation
    public static final double TURRET_ENCODER_CPR = 537.7 * GEAR_RATIO; // ≈ 1891.6 ticks per turret rotation

    // limits the turret's use of abs encoder beyond this area
    public static final double TURRET_MAX_ANGLE = 180.0; // deg
    public static final double TURRET_PID_TOLERANCE = 0.5; //deg
    public static final double TURRET_SAFE_ZONE = 160;

    // for relocalizing turret
    private double TURRET_HOME_OFFSET = 0;
    public double MANUAL_OFFSET = 0;


    public Turret() {
        turretAnglePID.setTolerance(TURRET_PID_TOLERANCE);
    }

    public void loadAngle(double angle) {
        TURRET_HOME_OFFSET = angle;
        robot.shooterTurret.stopAndResetEncoder();
    }

    @Override
    public void periodic() {
        // always use quadrature
        boolean shouldRelocalizeTurret = Math.abs(robot.analogEncoder.getCurrentPosition()) > TURRET_SAFE_ZONE;
        final double quadratureAngle = calculateAngleFromEncoder();
        final double absAngle = robot.analogEncoder.getCurrentPosition();
        lastAngle = quadratureAngle;

        // ensure that we're safe, not moving, and have good health
        if (!shouldRelocalizeTurret && Math.abs(robot.shooterTurret.getCorrectedVelocity()) < TURRET_ENCODER_CPR / 360.0 && robot.getVoltage() > 12) {
            double rawQuadAngle =
                    (robot.shooterTurret.encoder.getPosition()
                            / TURRET_ENCODER_CPR) * 360.0;
            double error = absAngle - rawQuadAngle;
            TURRET_HOME_OFFSET = (TURRET_HOME_OFFSET * 0.9) + (error * 0.1);
        }

        robot.flightRecorder.addLine("==========TURRET===========");
        robot.flightRecorder.addData("absolute encoder", absAngle);
        robot.flightRecorder.addData("quadrature angle", quadratureAngle);
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

            // resetting encoder (does nothing atm)
            case HOMING: {
                pendingMode = Mode.PINPOINT;
                break;
            }

            // OPTION A: use pinpoint to aim turret
            case PINPOINT: {
                if (lastMode != Mode.PINPOINT) {
                    turretAnglePID.reset();
                    setpointFilter.reset();
                }

                double predictedLeadOffset = Math.toDegrees(robot.drive.getTangentVelocityToGoal() * PREDICT_FACTOR);

                // filter our target
                double rawTarget = robot.drive.getAimTarget().heading;
                setpointFilter.updateValue(rawTarget);

                double filteredTarget = setpointFilter.getFilteredOutput() - predictedLeadOffset + MANUAL_OFFSET;

                // constrain our angles
                double constrainedAngleDeg = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, filteredTarget));
                double error = constrainedAngleDeg - quadratureAngle;
                double power = turretAnglePID.calculate(quadratureAngle, constrainedAngleDeg);
                if (Math.abs(error) > TURRET_PID_SWITCH) {
                    power = TURRET_APPROACH_kP * error;
                }

                // absolute limit (idk if this helps but should)
                if (isAtTarget() || (power > 0 && quadratureAngle > TURRET_MAX_ANGLE) || (power < 0 && quadratureAngle < -TURRET_MAX_ANGLE)) {
                    robot.shooterTurret.set(0.0);
                    break;
                }

                robot.shooterTurret.set(power);
                break;
            }
            // Debug purposes only
            case DEBUG: {
                double power = turretAnglePID.calculate(quadratureAngle, targetAngle);
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

    public boolean isAtTarget() {
        return turretAnglePID.atSetPoint();
    }

    private double calculateAngleFromEncoder() {
        // results in 90 <---- 0 -----> -90
        double encoderPos = robot.shooterTurret.encoder.getPosition();
        return TURRET_HOME_OFFSET + (encoderPos / TURRET_ENCODER_CPR) * 360.0;
    }
}
