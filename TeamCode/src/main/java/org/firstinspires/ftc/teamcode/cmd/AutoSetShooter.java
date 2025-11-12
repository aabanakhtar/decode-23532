package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class AutoSetShooter extends CommandBase {
    DuneStrider robot = DuneStrider.get();

    public AutoSetShooter() {
        addRequirements(robot.shooter);
    }

    @Override
    public void initialize() {
        robot.shooter.setMode(Shooter.Mode.DYNAMIC);
    }

    @Override
    public boolean isFinished() {
        return robot.shooter.isAtTargetVelo();
    }
}
