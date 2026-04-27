package org.firstinspires.ftc.teamcode.cmd;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import java.util.function.BooleanSupplier;

/*
Static class containing commonly used commands
 */
@Config
public class Commandlet {
    private static final DuneStrider dunestrider = DuneStrider.get();
    public static long TRANSFER_DELAY = (long) 900.0;

    public static Command If(Command toRunTrue, Command toRunFalse, BooleanSupplier b) {
        return new ConditionalCommand(toRunTrue, toRunFalse, b);
    }

    public static Command go(Follower follower, PathChain p, double maxPow) {
        return new FollowPathCommand(follower, p, maxPow);
    }

    public static Command run(Runnable r) {
        return new InstantCommand(r);
    }

    public static Command intakeSet(Intake.Mode mode) {
        return run(() -> dunestrider.intake.setMode(mode));
    }

    public static Command shootTeleOp() {
        return new SequentialCommandGroup(
                run(() -> dunestrider.shooter.setMode(Shooter.Mode.DYNAMIC)),
                run(() -> dunestrider.intake.openLatch()),
                new ConditionalCommand(
                        waitFor(900),
                        waitFor(1000),
                        () -> dunestrider.drive.follower.getPose().getY() > 80
                ),
                // run the intake
                new ParallelCommandGroup(
                        intakeSet(Intake.Mode.INGEST),
                        waitFor(TRANSFER_DELAY)
                ),
                intakeSet(Intake.Mode.OFF),
                // turn off after doing everything
                run(() -> DuneStrider.get().shooter.setIdle()),

                waitFor(200),
                run(() -> dunestrider.intake.closeLatch())
        );
    }


    public static Command shoot(long transfer_delay) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    // open the latch
                    waitFor((long)Intake.INTAKE_LATCH_DELAY),
                    run(() -> dunestrider.intake.openLatch())
                ),
                // run the intake
                new ParallelCommandGroup(
                    intakeSet(Intake.Mode.INGEST),
                    waitFor(transfer_delay)
                ),
                // turn off after doing everything
                run(() -> DuneStrider.get().shooter.setIdle()),
                intakeSet(Intake.Mode.OFF),
                run(() -> dunestrider.intake.closeLatch())
        );
    }

    public static Command shootFar(long transfer_delay) {
        // TODO: use distance sensors to gauge success
        return new SequentialCommandGroup(
                run(() -> Intake.INGEST_MOTOR_SPEED = 0.8),
                new ParallelCommandGroup(
                        // open the latch
                        waitFor((long) Intake.INTAKE_LATCH_DELAY),
                        run(() -> dunestrider.intake.openLatch())
                ),
                // run the intake
                new ParallelCommandGroup(
                        intakeSet(Intake.Mode.INGEST),
                        waitFor(transfer_delay)
                ),
                // turn off after doing everything
                run(() -> DuneStrider.get().shooter.setIdle()),
                intakeSet(Intake.Mode.OFF),
                run(() -> dunestrider.intake.closeLatch()),
                run(() -> Intake.INGEST_MOTOR_SPEED = 1)
        );
    }

    public static Command fork(Command a, Command b) {
        return new ParallelCommandGroup(a, b);
    }

    public static Command waitFor(long duration_ms) {
        return new WaitCommand(duration_ms);
    }

    public static Command nothing() {
        return run(() -> {});
    }
}
