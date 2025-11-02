package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.go;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.seq;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.CONTROL1_SCORE_ROW1;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.CONTROL1_SCORE_ROW2;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.CONTROL1_SCORE_ROW3;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE2;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE3;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.END_LINEUP_START_INTAKE;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.END_LINEUP_START_INTAKE2;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.END_LINEUP_START_INTAKE3;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.START_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.GoalSidePoses.UNIVERSAL_SCORE_TARGET;
import static org.firstinspires.ftc.teamcode.opmode.GlobalAutonomousPoses.heading;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
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
    public static double INTAKE_RECOLLECT_DELAY = 1000.0;

    private DuneStrider robot;
    public PathChain shootPreload;
    public PathChain lineUpRow1, lineUpRow2, lineUpRow3;
    public PathChain intakeRow1, intakeRow2, intakeRow3;
    public PathChain scoreRow1, scoreRow2, scoreRow3;
    public PathChain park;

    private double nRows = 0;
    private Prompter prompter;

    @Override
    public void init() {
        prompter = new Prompter(this);
        prompter.prompt("alliance", new OptionPrompt<>("Alliance select", DuneStrider.Alliance.RED, DuneStrider.Alliance.BLUE));
        prompter.prompt("rows", new ValuePrompt("Rows select", 0.0, 3.0, 0.0, 1.0));

        prompter.onComplete(() -> {
            DuneStrider.alliance = prompter.get("alliance");
            nRows = prompter.get("rows");
        });

        robot = DuneStrider.get().init(START_PRELOAD.setHeading(0), hardwareMap, telemetry);
        Follower follower = robot.drive.follower;
        buildPathChains(follower);


        CommandScheduler.getInstance().schedule(
                seq(
                        execPreload(),
                        If(
                                execRow1(),
                                nothing(),
                                () -> nRows >= 1
                        ),
                        If(
                                execRow2(),
                                nothing(),
                                () -> nRows >= 2
                        ),
                        If(
                                execRow3(),
                                nothing(),
                                () -> nRows >= 3
                        )
                )
        );
    }

    @Override
    public void init_loop() {
        prompter.run();
    }

    @Override
    public void loop() {
        robot.endLoop();
    }
    private Command execPreload() {
        return go(robot.drive.follower, shootPreload, 1.0);
    }

    private Command execRow1() {
        return seq(
                // RUN INTAKE WITH ALIGN
                go(robot.drive.follower, lineUpRow1, 1.0).
                        alongWith(intakeSet(Intake.Mode.INGEST)),
                go(robot.drive.follower, intakeRow1, 0.6),
                waitFor((long)INTAKE_RECOLLECT_DELAY),
                go(robot.drive.follower, scoreRow1, 1.0)
                        .alongWith(intakeSet(Intake.Mode.OFF))
        );
    }

    private Command execRow2() {
        return seq(
            go(follower, lineUpRow2, 1.0)
                    .alongWith(intakeSet(Intake.Mode.INGEST)),
            go(follower, intakeRow2, 0.6),
            waitFor((long)INTAKE_RECOLLECT_DELAY),
            go(follower, scoreRow2, 1.0)
                    .alongWith(intakeSet(Intake.Mode.OFF))
        );
    }

    private Command execRow3() {
        return seq(
                // RUN INTAKE WITH ALIGN
                go(robot.drive.follower, lineUpRow3, 1.0).
                        alongWith(intakeSet(Intake.Mode.INGEST)),
                go(robot.drive.follower, intakeRow3, 0.6),
                waitFor((long)INTAKE_RECOLLECT_DELAY),
                go(robot.drive.follower, scoreRow3, 1.0)
                        .alongWith(intakeSet(Intake.Mode.OFF))
        );
    }

    private void buildPathChains(Follower follower) {
        shootPreload = follower
                .pathBuilder()
                .addPath(new BezierLine(START_PRELOAD, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(0, heading(-45))
                .build();

        lineUpRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE))
                .setLinearHeadingInterpolation(heading(-45), heading(180))
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_LINEUP_START_INTAKE, END_INTAKE_START_SCORE))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(END_INTAKE_START_SCORE, CONTROL1_SCORE_ROW1, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(heading(180), heading(-45))
                .build();


        lineUpRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE2)
                )
                .setLinearHeadingInterpolation(heading(-45), heading(180))
                .build();

        intakeRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(END_LINEUP_START_INTAKE2, END_INTAKE_START_SCORE2)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        scoreRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE2,
                                CONTROL1_SCORE_ROW2,
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        lineUpRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE3)
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        intakeRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(END_LINEUP_START_INTAKE3, END_INTAKE_START_SCORE3)
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE3,
                                CONTROL1_SCORE_ROW3,
                                UNIVERSAL_SCORE_TARGET

                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(-45))
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(UNIVERSAL_SCORE_TARGET, new Pose(53.614, 100))
                )
                .setLinearHeadingInterpolation(heading(-45), heading(0))
                .build();
    }
}