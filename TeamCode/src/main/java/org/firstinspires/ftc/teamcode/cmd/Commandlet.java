package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

/*
Static class containing commonly used commands
 */
public class Commandlet {
    private static final DuneStrider dunestrider = DuneStrider.get();

    public static Command intakeSet(Intake.Mode mode) {
        return new InstantCommand(() -> dunestrider.intake.setMode(mode));
    }

    public static Command homeTurret() {
        return new HomeTurretCommand();
    }


    public static Command waitFor(long duration_ms) {
        return new WaitCommand(duration_ms);
    }

    public static Command nothing() {
        return new InstantCommand(() -> {});
    }
}
