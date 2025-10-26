package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Shooter extends SubsystemBase {
    public enum Mode {
        RAW,
        VELOCITY
    }

    public static boolean tuning = false;
    public static Mode mode = Mode.RAW;
    public static double targetVelocityTicks = 0.0;
    public static double targetRawPower = 0.0;

    public static double kV = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double tolerance = 100.0;
    private final PIDFController flywheelVelocityPID = new PIDFController(kP, kI, kD, kV);

    public static final InterpLUT distToVeloLUT;

    // initialize this thing to persist as is
    static {
        distToVeloLUT = new InterpLUT();
        distToVeloLUT.add(0, 0);
        distToVeloLUT.add(1, 0);
        // to do: add
        distToVeloLUT.createLUT();
    }

    public Shooter() {
        flywheelVelocityPID.setTolerance(tolerance);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case RAW:
                rawMode();
                break;
            case VELOCITY:
                velocityMode();
                break;
            default:
                break;
        }
    }

    public void setMode(Mode mode) {
        Shooter.mode = mode;
    }

    public double getOptimalVelocityForDist(double distance_ft) {
        return distToVeloLUT.get(distance_ft);
    }

    public void setVelocity(double velo) {
        mode = Mode.VELOCITY;
        targetVelocityTicks = velo;
    }

    public void setPower(double pow) {
        mode = Mode.RAW;
        targetRawPower = pow;
    }

    public boolean isAtTargetVelocity() {
        return flywheelVelocityPID.atSetPoint();
    }

    private void velocityMode() {
        if (tuning) {
            flywheelVelocityPID.setPIDF(kP, kI, kD, kV);
            flywheelVelocityPID.setTolerance(tolerance);
        }

        // set the target velocity
        DuneStrider robot = DuneStrider.get();
        double currentVelocity = robot.shooterLeft.getCorrectedVelocity();
        double output = flywheelVelocityPID.calculate(currentVelocity, targetVelocityTicks);

        robot.shooterLeft.set(output);
        robot.shooterRight.set(output);
        // log useful info
        robot.telemetry.addLine("========SHOOTER========");
        robot.telemetry.addData("Raw encoder:", robot.shooterLeft.encoder.getPosition());
        robot.telemetry.addData("Flywheel Target velocity", targetVelocityTicks);
        robot.telemetry.addData("Flywheel Current velocity", currentVelocity);
        robot.telemetry.addData("Flywheel raw power output", output);
    }

    private void rawMode() {
        DuneStrider robot = DuneStrider.get();
        robot.shooterLeft.set(targetRawPower);
        robot.shooterRight.set(targetRawPower);
    }
}
