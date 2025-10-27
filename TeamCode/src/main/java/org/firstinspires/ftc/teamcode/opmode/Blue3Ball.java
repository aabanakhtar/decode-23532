package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;

import org.firstinspires.ftc.teamcode.cmd.SetShooter;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@Configurable
@TeleOp(name = "Blue Top Side 12 Ball", group = "auto")
public class Blue3Ball extends OpMode {
    public PathChain startToShootPreload;

    public static Pose startPosition = new Pose(34, 136.5);
    public static Pose shootPreload = new Pose(34, 120.5);


    // Define poses
    private DuneStrider robot;

    @Override
    public void init() {
        robot = DuneStrider.get().init(startPosition.setHeading(Math.toRadians(180)), hardwareMap, telemetry);
        Follower follower = robot.drive.follower;
        startToShootPreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPosition, shootPreload)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        PathChain path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(34.000, 120.500),
                                new Pose(65.091, 78.000),
                                new Pose(29.091, 83.273)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .build();

        PathChain path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(29.091, 83.273),
                                shootPreload
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    new FollowPathCommand(robot.drive.follower, startToShootPreload),
                    new WaitCommand(2000),
                    new InstantCommand(() -> robot.intake.setMode(Intake.Mode.INGEST)),
                    new FollowPathCommand(robot.drive.follower, path2, 0.1),
                    new FollowPathCommand(robot.drive.follower, path3, 0.1)
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
