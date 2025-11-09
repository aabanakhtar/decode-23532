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
    LLResult lastResult = null;

    public MegaTagDetector(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        double robotHeading = Math.toDegrees(DuneStrider.get().drive.getPose().getHeading());
        limelight.updateRobotOrientation(robotHeading);

        // Fetch the latest data
        lastResult = limelight.getLatestResult();
        if (lastResult == null) {
            return;
        }

        if (!lastResult.isValid()) {
            lastResult = null;
            return;
        }

        Telemetry t = DuneStrider.get().telemetry;
        t.addData("TAG X OFFSET DEGREES", lastResult.getTx());
    }

    public LLResult getLastResult() {
        return lastResult;
    }

}
