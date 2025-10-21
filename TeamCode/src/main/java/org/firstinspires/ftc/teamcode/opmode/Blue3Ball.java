package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Blue3Ball extends OpMode {
    public PathChain startToShootPreload;

    public static Pose startPosition = new Pose(24.5, 136.5);
    public static Pose shootPreload = new Pose(40.23, 113.4);

    // Define poses
    private DuneStrider robot;

    @Override
    public void init() {
        robot = DuneStrider.get().init(new Pose(), hardwareMap, telemetry);
        Follower follower = robot.drive.follower;
        startToShootPreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPosition, shootPreload)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    new FollowPathCommand(robot.drive.follower, startToShootPreload)
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
