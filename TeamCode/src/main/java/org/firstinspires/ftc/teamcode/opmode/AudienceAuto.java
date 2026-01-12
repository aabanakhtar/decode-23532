package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.fork;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.go;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shoot;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.ACONTROL1_LINEUP_ROW3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.ACONTROL1_SCORE_ROW3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.AEND_INTAKE_ROW3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.ALINEUP_ROW3_END;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.AUNIVERSAL_SCORE_TARGET;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.heading;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.cmd.HomeTurret;
import org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@Autonomous(name = "Autonomous: 6 Artifact Non-configurable", group = "auto")
public class AudienceAuto extends OpMode {
    public static double TRANSFER_DELAY = 500.0;
    public static Pose START_POSE = new Pose(48, 7.3, GlobalAutonomousPoses.heading(90));
    private final DuneStrider robot = DuneStrider.get();
    private PathChain shootPreload, intakeRow3, scoreRow3, intakeHPZone, intakeHPZone2, scoreHPZone;

    @Override
    public void init() {
        Pose startPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ?
                START_POSE.setHeading(heading(90)) : START_POSE.mirror().setHeading(heading(90));
        robot.init(DuneStrider.Mode.AUTO, startPose, hardwareMap, telemetry);

        Follower follower = robot.drive.follower;
        if (DuneStrider.alliance == DuneStrider.Alliance.BLUE) buildPathChains(follower);
        else buildPathChainsRed(follower);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new HomeTurret(0.5),
                        execPreload(follower), execRow3(), execRowHP(), execRowHP())
        );
    }

    public Command execPreload(Follower follower) {
        return new SequentialCommandGroup(
                // prepare our intake for world class domination
                run(() -> Intake.INGEST_MOTOR_SPEED = 0.8),
                run(() -> robot.intake.closeLatch()),

                // prep shooter
                new InstantCommand(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),

                // drive up
                new FollowPathCommand(follower, shootPreload),
                waitFor(300),

                // shoot
                intakeSet(Intake.Mode.INGEST),
                run(() -> robot.intake.openLatch()),
                waitFor(1200),
                // de prepare
                run(() -> robot.intakeTubing.set(0)),
                run(() -> robot.shooter.setIdle())
        );
    }

    private Command execRow3() {
        return new SequentialCommandGroup(
                fork(
                        nothing(),
                        run(() -> robot.intake.closeLatch())
                ),

                // turn on the intake and eat up the balls
                intakeSet(Intake.Mode.INGEST),
                go(robot.drive.follower, intakeRow3, 1),
                waitFor(300),
                new InstantCommand(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                intakeSet(Intake.Mode.OFF),
                // go home and score
                go(robot.drive.follower, scoreRow3, 1),
                waitFor(300),
                // shoot
                intakeSet(Intake.Mode.INGEST),
                run(() -> robot.intake.openLatch()),
                waitFor((long)1200),

                // de prepare
                run(() -> robot.intakeTubing.set(0)),
                run(() -> robot.shooter.setIdle())
        );
    }

    private Command execRowHP() {
        return new SequentialCommandGroup(
                // HP ZONE ==============================================
                // turn on the intake and eat up the balls
                run(() -> robot.intake.closeLatch()),
                intakeSet(Intake.Mode.INGEST),
                go(robot.drive.follower, intakeHPZone, 1),
                waitFor(300),

                new InstantCommand(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                intakeSet(Intake.Mode.OFF),
                // go home and score
                go(robot.drive.follower, scoreHPZone, 1),
                waitFor(200),
                // shoot
                intakeSet(Intake.Mode.INGEST),
                run(() -> robot.intake.openLatch()),
                waitFor((long)2500),

                // de prepare
                run(() -> robot.intakeTubing.set(0)),
                run(() -> robot.shooter.setIdle()),
                run(() -> Intake.INGEST_MOTOR_SPEED = 1)
        );
    }

    @Override
    public void loop() {
        robot.endLoop();
    }

    public void buildPathChains(Follower follower) {
        shootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                START_POSE,
                                AUNIVERSAL_SCORE_TARGET
                        )
                )
                .setLinearHeadingInterpolation(heading(90), heading(-45))
                .build();

        intakeRow3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(AUNIVERSAL_SCORE_TARGET, new Pose(44, 37), AEND_INTAKE_ROW3)
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AEND_INTAKE_ROW3,
                                ACONTROL1_SCORE_ROW3,
                                AUNIVERSAL_SCORE_TARGET
                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(-45))
                .build();

        intakeHPZone = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AUNIVERSAL_SCORE_TARGET,
                                new Pose(58, 9),
                                new Pose(9, 8)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        intakeHPZone2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AUNIVERSAL_SCORE_TARGET,
                                new Pose(33, 36),
                                new Pose(9, 8)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreHPZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.5, 11),
                                AUNIVERSAL_SCORE_TARGET
                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(-45))
                .build();
    }

    public void buildPathChainsRed(Follower follower) {
        shootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                START_POSE.mirror(),
                                AUNIVERSAL_SCORE_TARGET.mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        heading(90),
                        mirrorHeading(heading(-45))
                )
                .build();

        intakeRow3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(ALINEUP_ROW3_END.mirror(), AEND_INTAKE_ROW3.mirror())
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                AEND_INTAKE_ROW3.mirror(),
                                ACONTROL1_SCORE_ROW3.mirror(),
                                AUNIVERSAL_SCORE_TARGET.mirror()
                        )
                )
                .setLinearHeadingInterpolation(mirrorHeading(heading(180)), mirrorHeading(heading(-45)))
                .build();
    }

    private double mirrorHeading(double heading) {
        return Math.PI - heading;
    }
}