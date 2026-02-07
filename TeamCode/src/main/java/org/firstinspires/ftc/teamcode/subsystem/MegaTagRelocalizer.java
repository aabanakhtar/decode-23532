package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.utilities.KalmanPoseEstimator;

public class MegaTagRelocalizer extends SubsystemBase {
    private final Limelight3A limelight;
    private final DuneStrider robot;

    public static boolean disabled = true;
    public static double MAX_SOURCE_DISPARITY = 6.0; // in
    public static double MAX_RELOCALIZE_VELOCITY = 2.0; // in / sec

    public static double LIMELIGHT_AXIS_COVARIANCE = 0.6458;
    public static double PINPOINT_AXIS_COVARIANCE = 0.0853;

    KalmanPoseEstimator xPoseEstimator = new KalmanPoseEstimator(0.5, 0.25, PINPOINT_AXIS_COVARIANCE, LIMELIGHT_AXIS_COVARIANCE);
    KalmanPoseEstimator yPoseEstimator = new KalmanPoseEstimator(0.5, 0.25, PINPOINT_AXIS_COVARIANCE, LIMELIGHT_AXIS_COVARIANCE);

    public MegaTagRelocalizer() {
        robot = DuneStrider.get();
        limelight = robot.limelight;
    }

    public void setEnabled(boolean enabled) {
        disabled = !enabled;
    }

    @Override
    public void periodic() {
        limelight.updateRobotOrientation(Math.toDegrees(robot.drive.getPose().getHeading() + Math.PI / 2));
        LLResult result = limelight.getLatestResult();

        double dt = robot.hubs.getDeltaTime();
        xPoseEstimator.updateKalmanUncertainty(dt);
        yPoseEstimator.updateKalmanUncertainty(dt);

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();
            Position position = botPose.getPosition();

            double x = DistanceUnit.INCH.fromMeters(position.x);
            double y = DistanceUnit.INCH.fromMeters(position.y);
            double heading = Math.toRadians(botPose.getOrientation().getYaw());
            Pose limelightPose = new Pose(y + 72, 72 - x, heading - Math.PI/2);

            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().setFill("blue").fillCircle(x, y, Math.toDegrees(heading));
            FtcDashboard.getInstance().sendTelemetryPacket(p);

            if (verifyLimelightPose(limelightPose, robot.drive.getPose()) && !disabled) {
                Pose pose = robot.drive.getPose();
                double xDrift = xPoseEstimator.getDriftKalman(pose.getX(), limelightPose.getX());
                double yDrift = yPoseEstimator.getDriftKalman(pose.getY(), limelightPose.getY());
                // filter based on drift
                pose = pose.minus(new Pose(xDrift, yDrift, 0));
                robot.drive.follower.setPose(pose);
            }
        }

    }

    public boolean verifyLimelightPose(Pose newEstimate, Pose current) {
        if (!(newEstimate.getX() < 144 && newEstimate.getY() < 144 && newEstimate.getX() > 0 && newEstimate.getY() > 0)) {
            return false;
        }

        if (newEstimate.distanceFrom(current) > MAX_SOURCE_DISPARITY) {
            return false;
        }

        return !(Math.abs(robot.drive.getVelocity().getMagnitude()) > MAX_RELOCALIZE_VELOCITY);
    }
}
