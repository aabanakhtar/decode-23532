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

    public static double INTAKE_LATCH_OPEN_POSITION = 0.4;
    public static double INTAKE_LATCH_CLOSE_POSITON = 0.67;
    public static double intakeLatchTargetPos = INTAKE_LATCH_OPEN_POSITION;
    public static double INTAKE_LATCH_DELAY = 300.0;

    public Intake() {

    }

    public void setMode(Mode mode) {
        Intake.mode = mode;
    }

    @Override
    public void periodic() {
        DuneStrider robot = DuneStrider.get();
        // update latch
        robot.latchServo.set(intakeLatchTargetPos);

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

    public void openLatch() {
        intakeLatchTargetPos = INTAKE_LATCH_OPEN_POSITION;
    }

    public void closeLatch() {
        intakeLatchTargetPos = INTAKE_LATCH_CLOSE_POSITON;
    }
}
