package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

public class SetShooter extends CommandBase {
    DuneStrider robot = DuneStrider.get();
    double targetVelocity = 0.0;

    public SetShooter(double targetVelocity) {
        addRequirements(robot.shooter);
        this.targetVelocity = targetVelocity;
    }

    @Override
    public void initialize() {
        robot.shooter.setVelocity(targetVelocity);
    }

    @Override
    public boolean isFinished() {
        return robot.shooter.isAtTargetVelocity();
    }
}
