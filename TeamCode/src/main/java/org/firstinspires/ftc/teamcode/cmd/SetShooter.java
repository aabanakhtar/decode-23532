package org.firstinspires.ftc.teamcode.cmd;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class SetShooter extends CommandBase {
    private final DuneStrider robot = DuneStrider.get();
    private final Shooter.Mode mode;
    private double target;
    private boolean autoCalculateDistance = false;

    // value = distance for VELOCITY in FEET 🦶
    // value = power for RAW
    public SetShooter(Shooter.Mode mode, double value, boolean autoCalculateDistance) {
        addRequirements(robot.shooter);
        this.mode = mode;
        this.target = value;
        this.autoCalculateDistance = autoCalculateDistance;
    }

    @Override
    public void initialize() {
        robot.shooter.setMode(mode);
    }

    @Override
    public void execute() {
        switch (mode) {
            case RAW:
                robot.shooter.setPower(target);
                break;
            case VELOCITY: {
                double targetVeloTicks;
                if (autoCalculateDistance) {
                    MecanumDrive.AimAtTarget aimAt = robot.drive.getShooterPositionPinpointRel();
                    // the function uses pinpoint inches so we have to convert
                    targetVeloTicks = robot.shooter.getOptimalVelocityForDist(aimAt.distance / 12.0); // convert to ft
                } else {
                    targetVeloTicks = robot.shooter.getOptimalVelocityForDist(target);
                }

                robot.shooter.setVelocity(targetVeloTicks);
                break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (mode == Shooter.Mode.RAW) return true;
        return robot.shooter.isAtTargetVelocity();
    }
}
