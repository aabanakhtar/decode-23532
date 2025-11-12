package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.concurrent.TimeUnit;

public class HomeTurret extends CommandBase {
    private final DuneStrider robot = DuneStrider.get();
    private final Timing.Timer timer ;

    public HomeTurret(double time) {
        addRequirements(DuneStrider.get().turret);
        timer = new Timing.Timer((long) time, TimeUnit.SECONDS);
    }

    @Override
    public void initialize() {
        robot.turret.setMode(Turret.Mode.HOMING);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        boolean cond = timer.done();
        if (cond) {
            robot.shooterTurret.stopAndResetEncoder();
            robot.turret.setMode(Turret.Mode.DYNAMIC);
        }

        return cond;
    }
}
