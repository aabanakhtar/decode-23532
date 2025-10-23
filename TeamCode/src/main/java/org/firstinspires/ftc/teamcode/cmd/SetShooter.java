package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class SetShooter extends CommandBase {
    private final DuneStrider robot = DuneStrider.get();
    private final Shooter.Mode mode;
    private final double target;

    // value = distance for VELOCITY
    // value = power for RAW
    public SetShooter(Shooter.Mode mode, double value) {
        addRequirements(robot.shooter);
        this.mode = mode;
        this.target = value;
    }

    @Override
    public void initialize() {
        robot.shooter.setMode(mode);
    }

    @Override
    public void execute() {
        switch (mode) {
            case RAW:
                robot.shooter.setPower(target);
                break;
            case VELOCITY: {
                double targetVeloTicks = robot.shooter.getOptimalVelocityForDist(target);
                robot.shooter.setVelocity(targetVeloTicks);
                break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (mode == Shooter.Mode.RAW) return true;
        return robot.shooter.isAtTargetVelocity();
    }
}
