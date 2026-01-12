package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;

@Config
public class Shooter extends SubsystemBase {
    public enum Mode {
        RAW,
        FIXED,
        DYNAMIC
    }

    public static boolean tuning = false;
    public static Mode mode = Mode.RAW;
    private final RunningAverageFilter velFilter = new RunningAverageFilter(10);

    public static double targetVelocityTicks = 0.0;
    public static double targetRawPower = 0.0;

    public static double IDLE_VELOCITY = -600.0;
    public static double kV = 0.0005118;
    public static double kP = 0.003;
    public static double kI = 0.0;
    public static double kD = 0.0004;
    public static double VELOCITY_TOLERANCE = 40.0;
    private final PIDFController flywheelVelocityPID = new PIDFController(kP, kI, kD, 0);

    DuneStrider robot = DuneStrider.get();

    public static double REGRESSION_A = 0.0116667;
    public static double REGRESSION_B = 0.0505952;
    public static double REGRESSION_C = 0.0989881;

    public static double shooterTimeRegression(double shotDistance) {
        if (shotDistance < 4.0 || shotDistance > 8.0) {
            return 0.0;
        }

        return REGRESSION_A * shotDistance * shotDistance
                + REGRESSION_B * shotDistance
                + REGRESSION_C;
    }

    // initialize this thing to persist as is
    public static final InterpLUT distToVeloLUT;
    static {
        distToVeloLUT = new InterpLUT();

        distToVeloLUT.add(-100, -1000);
        distToVeloLUT.add(4.6, -1000);
        distToVeloLUT.add(5, -1050);
        distToVeloLUT.add(5.7, -1075);
        distToVeloLUT.add(6, -1090);
        distToVeloLUT.add(6.5, -1090);
        distToVeloLUT.add(7.15, -1110);
        distToVeloLUT.add(7.7, -1140);
        distToVeloLUT.add(8.4, -1215);
        distToVeloLUT.add(9.4, -1250);
        // ======
        distToVeloLUT.add(11.6, -1385);
        distToVeloLUT.add(12, -1410);
        distToVeloLUT.add(12.5, -1470);
        distToVeloLUT.add(12.7, -1480);
        distToVeloLUT.add(13.07, -1500);
        // to do: add
        distToVeloLUT.createLUT();
    }

    public Shooter() {
        flywheelVelocityPID.setTolerance(VELOCITY_TOLERANCE);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case RAW:
                rawMode();
                break;
            case FIXED:
                velocityMode();
                break;
            case DYNAMIC:
                dynamicMode();
                break;
            default:
                break;
        }

        TelemetryManager p = PanelsTelemetry.INSTANCE.getTelemetry();
        p.addData("Target Velocity", targetVelocityTicks);
        p.addData("Current Velocity", velFilter.getFilteredOutput());
        p.update();

        velFilter.updateValue(robot.shooterLeft.getCorrectedVelocity());

        logData();
    }

    public void setMode(Mode mode) {
        Shooter.mode = mode;
    }

    public double getOptimalVelocityForDist(double distance_ft) {
        return distToVeloLUT.get(distance_ft);
    }

    public void setVelocity(double velo) {
        mode = Mode.FIXED;
        targetVelocityTicks = velo;
    }

    public void setPower(double pow) {
        mode = Mode.RAW;
        targetRawPower = pow;
    }

    public void setIdle() {
        setVelocity(IDLE_VELOCITY);
    }

    private void velocityMode() {
        if (tuning) {
            flywheelVelocityPID.setPIDF(kP, kI, kD, 0);
            flywheelVelocityPID.setTolerance(VELOCITY_TOLERANCE);
        }

        double currentVelocity = velFilter.getFilteredOutput();
        // voltage comp is necessary to prevent the bot from tweaking towards the end of the match
        double output = flywheelVelocityPID.calculate(currentVelocity, targetVelocityTicks) * robot.getVoltageFeedforwardConstant() +
                kV * targetVelocityTicks * robot.getVoltageFeedforwardConstant();

        robot.shooterLeft.set(output);
        robot.shooterRight.set(output);
    }

    private void dynamicMode() {
        double currentVelocity = velFilter.getFilteredOutput();
        // clip to a distance
        double distanceToGoal = Range.clip(robot.drive.getAimTarget().distance, 1.0, 12.0);
        double optimalVelocityForDist = getOptimalVelocityForDist(distanceToGoal);
        double output = flywheelVelocityPID.calculate(currentVelocity, optimalVelocityForDist) * robot.getVoltageFeedforwardConstant()
                + kV * optimalVelocityForDist * robot.getVoltageFeedforwardConstant();

        robot.shooterLeft.set(output);
        robot.shooterRight.set(output);
    }

    private void rawMode() {
        robot.shooterLeft.set(targetRawPower);
        robot.shooterRight.set(targetRawPower);
    }

    private void logData() {
        robot.flightRecorder.addLine("========SHOOTER========");
        robot.flightRecorder.addData("Flywheel Target velocity", targetVelocityTicks);
        robot.flightRecorder.addData("Flywheel Current velocity", robot.shooterLeft.encoder.getCorrectedVelocity());
        robot.flightRecorder.addData("Flywheel raw power output", robot.shooterLeft.get());
    }

    public boolean isAtTargetVelocity() {
        return flywheelVelocityPID.atSetPoint();
    }

}
