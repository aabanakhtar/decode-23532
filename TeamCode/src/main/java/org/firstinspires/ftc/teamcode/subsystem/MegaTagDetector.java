package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

public class MegaTagDetector extends SubsystemBase {
    private final Limelight3A limelight;
    Pose lastPose = null;

    public MegaTagDetector(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        double robotHeading = Math.toDegrees(DuneStrider.get().drive.getPose().getHeading());
        limelight.updateRobotOrientation(robotHeading);
        // Fetch the latest data
        LLResult latestResult = limelight.getLatestResult();
        if (latestResult == null) {
            return;
        }

        double xPos = 0, yPos = 0;
        if (latestResult.isValid()) {
            Pose3D mtPose = latestResult.getBotpose_MT2();
            if (mtPose != null) {
                xPos = mtPose.getPosition().x + 72.0;
                yPos = mtPose.getPosition().y + 72.0;
                lastPose = new Pose(xPos, yPos, Math.toRadians(robotHeading));
            }
        }

        Telemetry t = DuneStrider.get().telemetry;
        t.addLine("=======MT2 Detector=======");
        t.addData("Latest Pose X:", xPos);
        t.addData("Latest Pose Y:", yPos);
    }

    // returns the last detected pose from the pose estimator
    public Pose getDetectedPose() {
        return lastPose;
    }
}
