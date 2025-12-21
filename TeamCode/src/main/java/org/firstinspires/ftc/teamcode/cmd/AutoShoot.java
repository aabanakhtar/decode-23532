package org.firstinspires.ftc.teamcode.cmd;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import java.util.concurrent.TimeUnit;

@Configurable
public class AutoShoot extends CommandBase {
    private final DuneStrider robot = DuneStrider.get();
    private final Timing.Timer timer;

    public AutoShoot(double shootTime) {
        addRequirements(robot.shooter, robot.intake);
        timer = new Timing.Timer((long)(shootTime * 1000), TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize() {
        timer.start();
        robot.shooter.setMode(Shooter.Mode.DYNAMIC);
    }

    @Override
    public void execute() {
        if (robot.shooter.isAtTargetVelocity()) {
            robot.intake.setMode(Intake.Mode.INGEST);
        } else {
            robot.intake.setMode(Intake.Mode.OFF);
        }
    }

    @Override
    public boolean isFinished() {
        boolean cond = timer.done();
        if (cond) {
            robot.shooter.setIdle();
        }

        return cond;
    }
}
