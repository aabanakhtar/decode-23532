package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Shooter extends SubsystemBase {
    public enum Mode {
        RAW,
        VELOCITY
    }

    public static Mode mode = Mode.RAW;
    public static boolean tuning = false;
    public static double targetVelocityTicks = 0.0;
    public static double targetRawPower = 0.0;

    public static double kV = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    private final PIDFController flywheelVelocityPID = new PIDFController(kP, kI, kD, kV);

    private final InterpLUT shooterDistanceToVelocity = new InterpLUT();

    public Shooter() {
        shooterDistanceToVelocity.add(0.0, 0.0);

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

    private void velocityMode() {
        if (tuning) {
            flywheelVelocityPID.setPIDF(kP, kI, kD, kV);
        }

        // set the target velocity
        DuneStrider robot = DuneStrider.get();
        double currentVelocity = robot.shooterLeft.getCorrectedVelocity();
        double output = flywheelVelocityPID.calculate(currentVelocity, targetVelocityTicks);

        robot.shooterLeft.set(output);
        robot.shooterRight.set(output);

        // log useful info
        robot.telemetry.addLine("========SHOOTER========");
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
