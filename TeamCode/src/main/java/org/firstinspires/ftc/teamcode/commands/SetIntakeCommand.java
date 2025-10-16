package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class SetIntakeCommand extends CommandBase {
    Intake subsystem;

    public SetIntakeCommand(Intake.Mode mode) {
        subsystem = DuneStrider.get().intake;
        addRequirements(subsystem);
    }
}
