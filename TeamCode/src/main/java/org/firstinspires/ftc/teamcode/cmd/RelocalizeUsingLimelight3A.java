package org.firstinspires.ftc.teamcode.cmd;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.MegaTagRelocalizer;

public class RelocalizeUsingLimelight3A extends CommandBase {
    private GamepadEx rumblePad;

    public RelocalizeUsingLimelight3A(GamepadEx rumblePad) {
        this.rumblePad = rumblePad;
        addRequirements(DuneStrider.get().eyes);
    }

    public void execute() {
        final DuneStrider robot = DuneStrider.get();
        LLResult result = robot.limelight.getLatestResult();

        if (!result.isValid()) {
            rumblePad.gamepad.rumbleBlips(4);
            return;
        }

        Pose3D botPose = result.getBotpose();
        Position position = botPose.getPosition();
        double x = DistanceUnit.INCH.fromMeters(position.x);
        double y = DistanceUnit.INCH.fromMeters(position.y);
        double heading = Math.toRadians(botPose.getOrientation().getYaw());
        Pose limelightPose = new Pose(y + 72, 72 - x, heading - Math.PI/2);

        if (robot.eyes.verifyLimelightPose(limelightPose)) {
            robot.drive.follower.setPose(limelightPose);
            rumblePad.gamepad.rumbleBlips(1);
        } else {
            rumblePad.gamepad.rumbleBlips(4);
        }
    }

    public boolean isFinished() {
        return true;
    }
}
