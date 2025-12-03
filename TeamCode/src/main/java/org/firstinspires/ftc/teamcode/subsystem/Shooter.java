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
    public static double kP = 0.007;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double VELOCITY_TOLERANCE = 40.0;

    private final PIDFController flywheelVelocityPID = new PIDFController(kP, kI, kD, 0);

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
        distToVeloLUT.add(6, -1260);
        distToVeloLUT.add(7, -1275);
        distToVeloLUT.add(9, -1480);
        distToVeloLUT.add(11, -1510);
        distToVeloLUT.add(100, -1550);
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

        double currentVelocity = robot.shooterLeft.getCorrectedVelocity();
        // voltage comp is necessary to prevent the bot from tweaking towards the end of the match
        double output = flywheelVelocityPID.calculate(currentVelocity, targetVelocityTicks) +
                kV * targetVelocityTicks * robot.getVoltageFeedforwardConstant();

        robot.shooterLeft.set(output);
        robot.shooterRight.set(output);
    }

    private void dynamicMode() {
        double currentVelocity = robot.shooterLeft.getCorrectedVelocity();
        // clip to a distance
        double distanceToGoal = Range.clip(robot.drive.getAimTarget().distance, 1.0, 12.0);
        double optimalVelocityForDist = getOptimalVelocityForDist(distanceToGoal);
        double output = flywheelVelocityPID.calculate(currentVelocity, optimalVelocityForDist)
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
        robot.flightRecorder.addData("Raw encoder:", robot.shooterLeft.encoder.getPosition());
        robot.flightRecorder.addData("Flywheel Target velocity", targetVelocityTicks);
        robot.flightRecorder.addData("Flywheel Current velocity", robot.shooterLeft.encoder.getCorrectedVelocity());
        robot.flightRecorder.addData("Flywheel raw power output", robot.shooterLeft.get());
    }

    public boolean isAtTargetVelo() {
        return flywheelVelocityPID.atSetPoint();
    }
}
