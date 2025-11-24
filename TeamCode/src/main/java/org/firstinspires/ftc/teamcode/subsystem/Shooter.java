package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Shooter extends SubsystemBase {
    public enum Mode {
        RAW,
        FIXED,
        DYNAMIC
    }

    public static boolean tuning = false;
    public static Mode mode = Mode.RAW;
    public static double targetVelocityTicks = 0.0;
    public static double targetRawPower = 0.0;
    public static double IDLE_VELOCITY = -600.0;
    public static double kV = 0.00045;
    public static double kP = 0.00505;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double tolerance = 40.0;
    private final PIDFController flywheelVelocityPID = new PIDFController(kP, kI, kD, kV);

    public static final InterpLUT distToVeloLUT;

    DuneStrider robot = DuneStrider.get();

    // initialize this thing to persist as is
    static {
        distToVeloLUT = new InterpLUT();
        distToVeloLUT.add(-100, 0);
        distToVeloLUT.add(0, -970);
        distToVeloLUT.add(2, -990);
        distToVeloLUT.add(3, -1000);
        distToVeloLUT.add(4, -1050);
        distToVeloLUT.add(5, -1050);
        distToVeloLUT.add(6, -1130);
        distToVeloLUT.add(7, -1210);
        distToVeloLUT.add(100, -1380);
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
            case FIXED:
                velocityMode();
                break;
            case DYNAMIC:
                dynamicMode();
                break;
            default:
                break;
        }

        robot.telemetry.addLine("========SHOOTER========");
        robot.telemetry.addData("Raw encoder:", robot.shooterLeft.encoder.getPosition());
        robot.telemetry.addData("Flywheel Target velocity", targetVelocityTicks);
        robot.telemetry.addData("Flywheel Current velocity", robot.shooterLeft.encoder.getCorrectedVelocity());
        robot.telemetry.addData("Flywheel raw power output", robot.shooterLeft.get());
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
            flywheelVelocityPID.setPIDF(kP, kI, kD, kV);
            flywheelVelocityPID.setTolerance(tolerance);
        }

        double currentVelocity = robot.shooterLeft.getCorrectedVelocity();
        double output = flywheelVelocityPID.calculate(currentVelocity, targetVelocityTicks);

        robot.shooterLeft.set(output);
        robot.shooterRight.set(output);
    }

    private void dynamicMode() {
        double currentVelocity = robot.shooterLeft.getCorrectedVelocity();
        // clip to a distance
        double distanceToGoal = Range.clip(robot.drive.getAimTarget().distance, 1.0, 12.0);
        double optimalVelocityForDist = getOptimalVelocityForDist(distanceToGoal);
        double output = flywheelVelocityPID.calculate(currentVelocity, optimalVelocityForDist);

        robot.shooterLeft.set(output);
        robot.shooterRight.set(output);
    }

    private void rawMode() {
        DuneStrider robot = DuneStrider.get();
        robot.shooterLeft.set(targetRawPower);
        robot.shooterRight.set(targetRawPower);
    }

    public boolean isAtTargetVelo() {
        return flywheelVelocityPID.atSetPoint();
    }
}
