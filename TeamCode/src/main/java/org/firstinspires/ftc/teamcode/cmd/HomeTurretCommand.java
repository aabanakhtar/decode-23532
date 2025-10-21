package org.firstinspires.ftc.teamcode.cmd;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class HomeTurretCommand extends CommandBase {
    private final DuneStrider robot = DuneStrider.get();


    public HomeTurretCommand() {
        addRequirements(DuneStrider.get().turret);
    }

    @Override
    public void init() {
        robot.turret.setMode(Turret.Mode.HOMING);
    }

    @Override
    public boolean isFinished() {
        return robot.turret.isAtHome();
    }
}
