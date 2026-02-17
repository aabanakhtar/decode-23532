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
    private final DuneStrider robot;

    public static boolean disabled = true;
    public static double MAX_RELOCALIZE_VELOCITY = 2.0; // in / sec

    public static double LIMELIGHT_AXIS_COVARIANCE = 0.3608;
    public static double PINPOINT_AXIS_COVARIANCE = 0.1853;

    KalmanPoseEstimator xPoseEstimator = new KalmanPoseEstimator(0.5, 0.25, PINPOINT_AXIS_COVARIANCE, LIMELIGHT_AXIS_COVARIANCE);
    KalmanPoseEstimator yPoseEstimator = new KalmanPoseEstimator(0.5, 0.25, PINPOINT_AXIS_COVARIANCE, LIMELIGHT_AXIS_COVARIANCE);

    public MegaTagRelocalizer() {
        robot = DuneStrider.get();
    }

    public void setEnabled(boolean enabled) {
        disabled = !enabled;
    }

    @Override
    public void periodic() {
        if (disabled) return;
        Pose arducamPose = robot.cam.getPedroPose();
        if (arducamPose != null && verifyLimelightPose(arducamPose))  {
                robot.drive.follower.setPose(arducamPose);
            }
        }

    public boolean verifyLimelightPose(Pose newEstimate) {
        if (!(newEstimate.getX() < 144 && newEstimate.getY() < 144 && newEstimate.getX() > 0 && newEstimate.getY() > 0)) {
            return false;
        }

        return !(Math.abs(robot.drive.getVelocity().getMagnitude()) > MAX_RELOCALIZE_VELOCITY);
    }
}
