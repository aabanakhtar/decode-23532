package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

public class MecanumDriveSubsystem extends SubsystemBase {
    public Follower follower;
    public static Pose pose;

    public MecanumDriveSubsystem(Pose startingPose) {
        this.follower = Constants.createFollower(DuneStrider.get().hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    @Override
    public void periodic() {
        pose = follower.getPose();
        DuneStrider robot = DuneStrider.get();
        robot.telemetry.addData("X:", pose.getX());
        robot.telemetry.addData("Y:", pose.getY());
        robot.telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        follower.update();
    }

    public void resetHeading(double newHeading) {
        follower.setPose(follower.getPose().setHeading(newHeading));
    }

    public void setTeleOpDrive(double forward, double strafe, double rotation) {
        follower.setTeleOpDrive(forward, strafe, rotation, true);
    }

    /**
     * Get the current estimated robot pose
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Get the current robot velocity
     */
    public Vector getVelocity() {
        return follower.getVelocity();
    }
}
