package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.concurrent.TimeUnit;

public class HomeTurret extends CommandBase {
    private final DuneStrider robot = DuneStrider.get();
    private final Timing.Timer timer = new Timing.Timer(3, TimeUnit.SECONDS);

    public HomeTurret() {
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
