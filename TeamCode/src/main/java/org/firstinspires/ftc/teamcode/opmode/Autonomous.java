package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Configurable
@TeleOp(name = "Autonomous", group = "auto")
public class Autonomous extends OpMode {
    private DuneStrider robot;

    public static final Pose UNIVERSAL_SCORE_TARGET = new Pose(53.179, 93.882);
    // preload points
    public static final Pose START_PRELOAD = new Pose(31.636, 136.515);
    // intake row 1
    public static final Pose END_LINEUP_START_INTAKE = new Pose(48.190, 89.904);
    public static final Pose END_INTAKE_START_SCORE = new Pose(12.360, 83.451);
    // score row 1 bezier control points
    public static final Pose CONTROL1_SCORE_ROW1 = new Pose(52.726, 78.235);

    public static final double HEAD_90 = Math.toRadians(90);
    public static final double HEAD_NEG_45 = Math.toRadians(135);
    public static final double HEAD_180 = Math.toRadians(180);

    public PathChain shootPreload;
    public PathChain lineUpRow1, lineUpRow2, lineUpRow3;
    public PathChain intakeRow1, intakeRow2, intakeRow3;
    public PathChain scoreRow1, scoreRow2, scoreRow3;
    public PathChain park;

    private double nRows = 0;

    @Override
    public void init() {
        Prompter prompter = new Prompter(this);
        prompter.prompt("alliance", new OptionPrompt<>("Alliance select", DuneStrider.Alliance.RED, DuneStrider.Alliance.BLUE));
        prompter.prompt("rows", new ValuePrompt("Rows select", 0.0, 3.0, 0.0, 1.0));

        prompter.onComplete(() -> {
            DuneStrider.alliance = prompter.get("alliance");
            nRows = prompter.get("rows");
        });

        robot = DuneStrider.get().init(START_PRELOAD.setHeading(0), hardwareMap, telemetry);
        Follower follower = robot.drive.follower;

        shootPreload = follower
                .pathBuilder()
                .addPath(new BezierLine(START_PRELOAD, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(0, HEAD_NEG_45)
                .build();

        lineUpRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE))
                .setLinearHeadingInterpolation(HEAD_NEG_45, HEAD_180)
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_LINEUP_START_INTAKE, END_INTAKE_START_SCORE))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(END_INTAKE_START_SCORE, CONTROL1_SCORE_ROW1, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(HEAD_180, HEAD_NEG_45)
                .build();


        lineUpRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(53.179, 93.882), new Pose(53.377, 60.020))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        intakeRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(53.377, 60.020), new Pose(15.420, 59.308))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.420, 59.308),
                                new Pose(74.965, 60.731),
                                new Pose(53.140, 93.944)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        lineUpRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(53.140, 93.944), new Pose(53.140, 36.534))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        intakeRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(53.140, 36.534), new Pose(15.183, 36.059))
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.183, 36.059),
                                new Pose(80.659, 31.789),
                                new Pose(53.377, 93.944)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(53.377, 93.944), new Pose(53.614, 137.595))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, shootPreload),
                        waitFor(1000),
                        // intake row 1
                        new ConditionalCommand (
                                new SequentialCommandGroup(
                                        new FollowPathCommand(follower, lineUpRow1),
                                        intakeSet(Intake.Mode.INGEST),
                                        new FollowPathCommand(follower, intakeRow1),
                                        waitFor(1000),
                                        new FollowPathCommand(follower, scoreRow1),
                                        waitFor(1000),
                                        new FollowPathCommand(follower, lineUpRow2),
                                        new FollowPathCommand(follower, intakeRow2),
                                        waitFor(1000),
                                        new FollowPathCommand(follower, scoreRow2),
                                        waitFor(1000),
                                        new FollowPathCommand(follower, lineUpRow3),
                                        new FollowPathCommand(follower, intakeRow3),
                                        waitFor(1000),
                                        new FollowPathCommand(follower, scoreRow3),
                                        waitFor(1000),
                                        new FollowPathCommand(follower, park)
                                ),
                                nothing(),
                                () -> true
                        )
                )
        );
    }

    @Override
    public void loop() {
        robot.endLoop();
    }
}