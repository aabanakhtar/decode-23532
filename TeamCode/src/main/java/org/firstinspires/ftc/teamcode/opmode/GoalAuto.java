package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.fork;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.go;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shoot;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_GATE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE_HP_ZONE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.GATE_CONTROL_POINT;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.GATE_OPEN;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT4;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT4_0;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_SCORE_R2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.SCORE_CONTROL_POINT3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.START_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.UNIVERSAL_SCORE_TARGET;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.heading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.cmd.HomeTurret;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.w3c.dom.Node;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Global Goal Autonomous (12 max compat)", group = "auto", preselectTeleOp = "TeleOp")
public class GoalAuto extends OpMode {
    public static double TRANSFER_DELAY = 650.0;
    public static double INTAKE_LUCKY_CHARM = 500.0;
    public static long INTAKE_DELAY = 0;
    public static double GATE_BRAKE_START = 0.7;
    public static long GATE_DURATION = 800;
    public static double GATE_HEADING = 160;
    public static double BRAKE_STRENGTH = 1.0;
    public static double TVALUE_CONSTRAINT = 1;
    public static Pose END_GATE_POSE2 = new Pose(8, 60.8);
    public static double MAX_SPD = 0.1;
    public static double BRAKE_THRESH = 0.87;
    public static double SLOWDOWN_SPEED = 0.2;
    public static double GATE_SPEED = 0.8;
    public static double R2_MAIN_SPEED = 0.7;
    public static double PRELOAD_SLOWDOWN_THRESH = 0.6;

    private DuneStrider robot;
    private PathChain shootPreload;
    private PathChain intakeRow1, intakeRow2, intakeRow3;
    private PathChain scoreRow1, scoreRow2, scoreRow3;
    private PathChain openGate, gateCycle, gateCycle2, shootGate;
    private PathChain parkRP;

    public static int nRows = 4;

    @Override
    public void init() {
        Pose startPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? START_PRELOAD.setHeading(0) : START_PRELOAD.mirror().setHeading(heading(180));

        robot = DuneStrider.get().init(DuneStrider.Mode.AUTO, startPose, hardwareMap, telemetry);
        robot.eyes.setEnabled(false);

        Follower follower = robot.drive.follower;
        follower.setMaxPowerScaling(MAX_SPD);

        if (DuneStrider.alliance == DuneStrider.Alliance.BLUE) buildPathChains(follower);
        else buildPathChainsRed(follower);

        // we ball
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new HomeTurret(0.15),
                        execPreload(),
                        execRow2(),
                        execRowGate(),
                        execRowGate(),
                        execRow1(),
                        go(follower, parkRP, 1),
                        run(() -> robot.shooter.setVelocity(0))
                )
        );
    }

    @Override
    public void init_loop() {
        telemetry.addLine("====DUNESTRIDER PRE-MATCH Config=====");
        telemetry.addData("|| > ALLIANCE:", DuneStrider.alliance.toString());
        telemetry.addData("|| > ROWS:", nRows);
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("GB MAX POWER: ", robot.drive.follower.getMaxPowerScaling());
        robot.endLoop();
    }

    private Command execPreload() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.openLatch()),
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                new FollowPathCommand(robot.drive.follower, shootPreload, true),
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRowGate() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                new FollowPathCommand(robot.drive.follower, gateCycle, 1.0),
                waitFor(GATE_DURATION),

                // let the intake regen
                fork (
                        new SequentialCommandGroup(
                                waitFor((long)INTAKE_LUCKY_CHARM),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, shootGate, true, 1.0)
                        )
                ),

                // go home and score
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRowGate2() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                waitFor(INTAKE_DELAY),

                new FollowPathCommand(robot.drive.follower, gateCycle2, 1.0),
                waitFor(GATE_DURATION),

                // let the intake regen
                fork (
                        new SequentialCommandGroup(
                                waitFor((long)INTAKE_LUCKY_CHARM),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, shootGate, 1)
                        )
                ),

                // go home and score
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRow1() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                new FollowPathCommand(robot.drive.follower, intakeRow1, false, 1.0),

                waitFor(INTAKE_DELAY),
                // let the intake regen
                fork (
                        new SequentialCommandGroup(
                                waitFor((long)INTAKE_LUCKY_CHARM),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, scoreRow1, true, 1.0)
                        )
                ),

                // go home and score
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRow2() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),
                // turn on the intake and eat up the balls
                intakeSet(Intake.Mode.INGEST),
                new FollowPathCommand(robot.drive.follower, intakeRow2, true, 1),
                waitFor(INTAKE_DELAY),

                fork(
                        new SequentialCommandGroup(
                                waitFor((long)INTAKE_LUCKY_CHARM),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, scoreRow2, true, 1)
                        )
                ),

                // go home and score
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRow3() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),

                // turn on the intake and eat up the balls
                intakeSet(Intake.Mode.INGEST),
                go(robot.drive.follower, intakeRow3, 1),
                waitFor(INTAKE_DELAY),

                fork(
                    new SequentialCommandGroup(
                            waitFor((long)TRANSFER_DELAY),
                            intakeSet(Intake.Mode.OFF)
                    ),
                    new SequentialCommandGroup(
                            run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                            new FollowPathCommand(robot.drive.follower, scoreRow3, true, 1)
                    )
                ),

                // go home and score
                shoot((long) TRANSFER_DELAY)
        );
    }

    private void buildPathChains(Follower follower) {
        shootPreload = follower
                .pathBuilder()
                .addPath(new BezierCurve(START_PRELOAD, new Pose(49, 113), UNIVERSAL_SCORE_TARGET))
                .addParametricCallback(PRELOAD_SLOWDOWN_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setTangentHeadingInterpolation()
                .setBrakingStart(0.7)
                .setBrakingStrength(BRAKE_STRENGTH)
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                            UNIVERSAL_SCORE_TARGET,
                            INTAKE_CONTROL_POINT,
                            END_INTAKE_START_SCORE
                        )
                )
                .addParametricCallback(0, () -> follower.setMaxPowerScaling(R2_MAIN_SPEED))
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_INTAKE_START_SCORE, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(heading(180), heading(-90))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        gateCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                UNIVERSAL_SCORE_TARGET,
                                new Pose(54, 55),
                                INTAKE_GATE
                        )
                )
                .addParametricCallback(0.1, () -> follower.setMaxPowerScaling(GATE_SPEED))
                .setTangentHeadingInterpolation()
                .setTValueConstraint(TVALUE_CONSTRAINT)
                .addPath(
                        new BezierLine(
                                INTAKE_GATE,
                                END_GATE
                        )
                )
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setConstantHeadingInterpolation(heading(GATE_HEADING))
                .build();

        gateCycle2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                UNIVERSAL_SCORE_TARGET,
                                new Pose(54, 55),
                                INTAKE_GATE
                        )
                )
                .setTangentHeadingInterpolation()
                .setTValueConstraint(TVALUE_CONSTRAINT)
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addPath(
                        new BezierLine(
                                INTAKE_GATE,
                                END_GATE_POSE2
                        )
                )
                .setConstantHeadingInterpolation(heading(GATE_HEADING))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        shootGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_GATE,
                                new Pose(52, 46),
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        intakeRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                UNIVERSAL_SCORE_TARGET,
                                INTAKE_CONTROL_POINT2,
                                END_INTAKE_START_SCORE2
                        )
                )
                .addParametricCallback(0.4, () -> follower.setMaxPowerScaling(R2_MAIN_SPEED))
                .setConstantHeadingInterpolation(heading(180))
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                            END_INTAKE_START_SCORE2,
                            GATE_CONTROL_POINT,
                            GATE_OPEN
                        )
                )
                .setConstantHeadingInterpolation(heading(180))
                .setBrakingStart(GATE_BRAKE_START)
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        scoreRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE2,
                                INTAKE_CONTROL_SCORE_R2,
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0.0, 0.8, HeadingInterpolator.constant(heading(180))),
                                new HeadingInterpolator.PiecewiseNode(0.3, 1, HeadingInterpolator.constant(heading(-90)))
                        )
                )
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        intakeRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                UNIVERSAL_SCORE_TARGET,
                                INTAKE_CONTROL_POINT3,
                                END_INTAKE_START_SCORE3
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        scoreRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE3,
                                SCORE_CONTROL_POINT3,
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        parkRP = follower.pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET, new Pose(48, 72)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(BRAKE_THRESH, () -> follower.setMaxPowerScaling(SLOWDOWN_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();
    }

    private void buildPathChainsRed(Follower follower) {

    }

    private double mirrorHeading(double heading) {
        return Math.PI - heading;
    }
}