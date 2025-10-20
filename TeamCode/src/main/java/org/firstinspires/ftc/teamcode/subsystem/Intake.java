package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Intake extends SubsystemBase {
    public enum Mode {
        INGEST,
        DISCARD,
        //MAINTAIN, // maintain posses
        OFF
    }

    public static Mode mode = Mode.OFF;

    public static double INGEST_MOTOR_SPEED = 0.0;
    public static double INGEST_SERVO_SPEED = 0.0;

    public static double DISCARD_MOTOR_SPEED = 0.0;
    public static double DISCARD_SERVO_SPEED = 0.0;

    public Intake() {

    }

    @Override
    public void periodic() {
        DuneStrider robot = DuneStrider.get();

        switch (mode) {
            case INGEST:
                robot.intakeTubing.set(INGEST_MOTOR_SPEED);
                robot.leftTransferWheel.set(INGEST_SERVO_SPEED);
                robot.rightTransferWheel.set(INGEST_SERVO_SPEED);
                break;

            case DISCARD:
                robot.intakeTubing.set(DISCARD_MOTOR_SPEED);
                robot.leftTransferWheel.set(DISCARD_SERVO_SPEED);
                robot.rightTransferWheel.set(DISCARD_SERVO_SPEED);
                break;

            case OFF:
                robot.intakeTubing.set(0.0);
                robot.leftTransferWheel.set(0.0);
                robot.rightTransferWheel.set(0.0);
                break;

            default:
                break;
        }
    }
}
