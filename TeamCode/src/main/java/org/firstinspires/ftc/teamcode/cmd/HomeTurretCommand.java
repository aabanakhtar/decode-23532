package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.concurrent.TimeUnit;

public class HomeTurretCommand extends CommandBase {
    private final DuneStrider robot = DuneStrider.get();
    private final Timing.Timer timer = new Timing.Timer(2, TimeUnit.SECONDS);

    public HomeTurretCommand() {
        addRequirements(DuneStrider.get().turret);
    }

    @Override
    public void initialize() {
        robot.turret.setMode(Turret.Mode.HOMING);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return robot.turret.isAtHome() || timer.done();
    }
}
