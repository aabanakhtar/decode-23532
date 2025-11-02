package org.firstinspires.ftc.teamcode.cmd;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.util.function.BooleanSupplier;

/*
Static class containing commonly used commands
 */
public class Commandlet {
    private static final DuneStrider dunestrider = DuneStrider.get();

    public static Command If(Command toRunTrue, Command toRunFalse, BooleanSupplier b) {
        return new ConditionalCommand(toRunTrue, toRunFalse, b);
    }

    public static Command go(Follower follower, PathChain p, double maxPow) {
        return new FollowPathCommand(follower, p, maxPow);
    }

    public static Command run(Runnable r) {
        return new InstantCommand(r);
    })

    public static Command seq(Command... commands) {
        return new SequentialCommandGroup(commands);
    }

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
