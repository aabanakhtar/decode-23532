package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.fork;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.go;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shoot;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shootFar;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto.INTAKE_RECOLLECTION_TIMEOUT;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto.PW_SCALE_BRAKE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto.PW_SCALE_PATH_SPEED;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto.SHOOTER_TRANSFER_DELAY;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto.mHBA;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto.mPBA;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.A_ROW3_END;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.A_ROw3_CONTROL;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.AudienceSidePoses.A_SCORE_TARGET;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.heading;

import com.acmerobotics.dashboard.config.Config;
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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Autonomous(name = "Autonomous: 6 Artifact Non-configurable", group = "auto")
@Config
public class AudienceAuto extends OpMode {
    public static double TRANSFER_DELAY = 1200.0;
    public static Pose START_POSE = new Pose(48, 7.3, GlobalAutonomousPoses.heading(90));
    private final DuneStrider robot = DuneStrider.get();

    private PathChain intakeRow3, scoreRow3, parkRP, hpZone, scoreHpZone;
    private Pose startPose;
    public static Pose hpPose = new Pose(11, 8);

    public static double FARSIDE_TRANSFER_DELAY = 1000;

    @Override
    public void init() {
        startPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ?
                START_POSE.setHeading(heading(90)) : START_POSE.mirror().setHeading(heading(90));
        robot.init(DuneStrider.Mode.AUTO, startPose, hardwareMap, telemetry);
        Follower follower = robot.drive.follower;
        buildPathChains(follower);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        execPreload(follower),
                        execRow3(),
                        execHPZone(),
                        go(follower, parkRP, 1.0)
        ));
    }

    public void stop() {
        Turret.offset_angle = 0;
    }

    public Command execPreload(Follower follower) {
        return new SequentialCommandGroup(
                // prepare our intake for world class domination
                run(() -> Intake.INGEST_MOTOR_SPEED = 0.6),
                run(() -> robot.intake.closeLatch()),

                // prep shooter
                new InstantCommand(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                waitFor(1000),
                shootFar(1000),
                run(() -> Intake.INGEST_MOTOR_SPEED = 1.0)
        );
    }

    private Command execRow3() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                new FollowPathCommand(robot.drive.follower, intakeRow3, false, 1.0),

                // let the intake regen
                fork (
                        new SequentialCommandGroup(
                                waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, scoreRow3, true, 1.0)
                        )
                ),

                // go home and score
                waitFor(1000),
                shootFar((long) FARSIDE_TRANSFER_DELAY)
        );
    }

    private Command execHPZone() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                run(() -> robot.drive.follower.setMaxPowerScaling(0.35)),
                new FollowPathCommand(robot.drive.follower, hpZone, false, 1.0),
                run(() -> robot.drive.follower.setMaxPowerScaling(1.0)),
                waitFor(1200),
                // let the intake regen
                fork (
                        new SequentialCommandGroup(
                                waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, scoreHpZone, true, 1.0)
                        )
                ),

                // go home and score
                waitFor(1000),
                shootFar((long) FARSIDE_TRANSFER_DELAY)
        );
    }

    @Override
    public void loop() {
        robot.endLoop();
    }

    public void buildPathChains(Follower follower) {
        intakeRow3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                mPBA(GlobalAutonomousPoses.AudienceSidePoses.A_ROw3_CONTROL),
                                mPBA(A_ROW3_END)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1))
                .build();

        scoreRow3 = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        mPBA(A_ROW3_END),
                        mPBA(A_ROw3_CONTROL),
                        mPBA(A_SCORE_TARGET)
                    )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1))
                .build();

        parkRP = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                mPBA(A_SCORE_TARGET),
                                mPBA(new Pose(56, 36))
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        hpZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                mPBA(A_SCORE_TARGET),
                                mPBA(hpPose)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreHpZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                mPBA(hpPose),
                                mPBA(A_SCORE_TARGET)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

    }

    private double mirrorHeading(double heading) {
        return Math.PI - heading;
    }
}