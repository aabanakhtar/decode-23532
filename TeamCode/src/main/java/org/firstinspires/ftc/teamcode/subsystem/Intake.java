package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Intake extends SubsystemBase {
    public enum Mode {
        INGEST,
        DISCARD,
        OFF
    }

    public static Mode mode = Mode.OFF;

    public static double INGEST_MOTOR_SPEED = 1.0;
    public static double DISCARD_MOTOR_SPEED = -1.0;

    public Intake() {

    }

    public void setMode(Mode mode) {
        Intake.mode = mode;
    }

    @Override
    public void periodic() {
        DuneStrider robot = DuneStrider.get();

        switch (mode) {
            case INGEST:
                robot.intakeTubing.set(INGEST_MOTOR_SPEED);
                break;

            case DISCARD:
                robot.intakeTubing.set(DISCARD_MOTOR_SPEED);
                break;

            case OFF:
                robot.intakeTubing.set(0.0);
                break;

            default:
                break;
        }
    }
}
