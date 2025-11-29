package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

import java.util.List;

public class HugeEyes extends SubsystemBase {
    private final Limelight3A limelight;
    private final DuneStrider robot;

    private @Nullable Double lastTx = null;
    private final static double MAX_ANGLE_TOLERANCE = 20.0;

    public HugeEyes() {
        robot = DuneStrider.get();
        limelight = robot.limelight;
    }

    @Override
    public void periodic() {
        limelight.updateRobotOrientation(Math.toDegrees(robot.drive.getPose().getHeading()));
        LLResult result = limelight.getLatestResult();

        robot.flightRecorder.addLine("=======LIMELIGHT=========");
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            LLResultTypes.FiducialResult chosenResult = null;

            for (LLResultTypes.FiducialResult fid : fiducials) {
                int tagID = fid.getFiducialId();

                if (DuneStrider.alliance == DuneStrider.Alliance.BLUE && tagID == 20) {
                    chosenResult = fid;
                    break;
                }
                else if (DuneStrider.alliance == DuneStrider.Alliance.RED && tagID == 24) {
                    chosenResult = fid;
                    break;
                }
            }

            if (chosenResult != null) {
                lastTx = chosenResult.getTargetXDegrees();
                robot.flightRecorder.addData("tx", -chosenResult.getTargetXDegrees());
                robot.flightRecorder.addData("ty", chosenResult.getTargetYDegrees());
            } else {
                lastTx = null;
                robot.flightRecorder.addLine("No suitable target found for limelight.");
            }

        } else {
            lastTx = null;
            robot.flightRecorder.addLine("Limelight returned null.");
        }
    }

    @Nullable
    public Double getLastTx() {
        return lastTx;
    }
}
