package org.firstinspires.ftc.teamcode.subsystem;


import android.util.Size;

import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ArduCam extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();
    private final VisionPortal visionPortal;
    private final AprilTagProcessor processor;

    private static final Position camPose = null;
    private static final YawPitchRollAngles orientation = null;

    public static int N_THREADS = 3;

    // Calibration Constants
    public static double FX = 516.3798424;
    public static double FY = 515.8231389;
    public static double CX = 328.1776587;
    public static double CY = 237.3745503;

    public static final double BLUE_TAG = 20;
    public static final double RED_TAG = 24;
    public static final double MAX_STALENESS = 1e8;

    private Pose lastDetection = null;
    private long lastReadStamp = 0;

    public ArduCam(CameraName webcam) {
        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setNumThreads(N_THREADS)
                .setLensIntrinsics(FX, FY, CX, CY)
                .setCameraPose(camPose, orientation)
                //.setCameraPose(new Position(), new YawPitchRollAngles())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawCubeProjection(true)
                .build();

        processor.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();
    }

    @Override
    public void periodic() {
        lastDetection = null;
        List<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.metadata != null && !d.metadata.name.contains("obelisk")) {
                if (d.id == BLUE_TAG || d.id == RED_TAG) {
                    lastReadStamp = System.nanoTime();
                    Position pos = d.robotPose.getPosition();
                    YawPitchRollAngles angles = d.robotPose.getOrientation();

                    double x = pos.x, y = pos.y, thetaDegrees = angles.getYaw();

                    lastDetection = new Pose(y + 72, 72 - x, Math.toRadians(thetaDegrees));
                    logData(lastDetection);
                }
            }
        }
    }

    @Nullable
    public Pose getPedroPose() {
        if (System.nanoTime() - lastReadStamp > MAX_STALENESS) {
            return null;
        }

        return lastDetection;
    }

    private void logData(Pose p) {
        robot.flightRecorder.addLine("===== ARDUCAM =====");
        robot.flightRecorder.addData("X", p.getX());
        robot.flightRecorder.addData("Y", p.getY());
        robot.flightRecorder.addData("THETA", Math.toDegrees(p.getHeading()));
    }
}
