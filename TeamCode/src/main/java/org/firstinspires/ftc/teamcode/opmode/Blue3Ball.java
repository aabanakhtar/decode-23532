package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Configurable
@TeleOp(name = "Blue Top Side 12 Ball", group = "auto")
public class Blue3Ball extends OpMode {
    private DuneStrider robot;

    public static final Pose UNIVERSAL_SCORE_TARGET = new Pose(53.179, 93.882);
    // preload points
    public static final Pose START_PRELOAD = new Pose(31.636, 136.515);
    // intake row 1
    public static final Pose END_LINEUP_START_INTAKE = new Pose(48.190, 83.904);
    public static final Pose END_INTAKE_START_SCORE = new Pose(12.360, 83.451);
    // score row 1 bezier control points
    public static final Pose CONTROL1_SCORE_ROW1 = new Pose(52.726, 78.235);

    public static final double HEAD_90 = Math.toRadians(90);
    public static final double HEAD_135 = Math.toRadians(135);
    public static final double HEAD_180 = Math.toRadians(180);

    public PathChain shootPreload;
    public PathChain lineUpRow1;
    public PathChain intakeRow1;
    public PathChain scoreRow1;

    @Override
    public void init() {
        robot = DuneStrider.get().init(START_PRELOAD.setHeading(HEAD_90), hardwareMap, telemetry);
        Follower follower = robot.drive.follower;

        shootPreload = follower
                .pathBuilder()
                .addPath(new BezierLine(START_PRELOAD, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(HEAD_90, HEAD_135)
                .build();

        lineUpRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE))
                .setLinearHeadingInterpolation(HEAD_135, HEAD_180)
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_LINEUP_START_INTAKE, END_INTAKE_START_SCORE))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(END_INTAKE_START_SCORE, CONTROL1_SCORE_ROW1, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(HEAD_180, HEAD_135)
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, shootPreload),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, lineUpRow1),
                        new InstantCommand(() -> robot.intake.setMode(Intake.Mode.INGEST)),
                        new FollowPathCommand(follower, intakeRow1, 0.1),
                        new FollowPathCommand(follower, scoreRow1)
                )
        );
    }

    @Override
    public void init_loop() {
        robot.endLoop();
    }

    @Override
    public void loop() {
        robot.endLoop();
    }
}