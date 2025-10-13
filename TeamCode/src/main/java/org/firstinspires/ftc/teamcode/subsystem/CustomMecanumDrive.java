package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

public class CustomMecanumDrive extends SubsystemBase {
    private final MecanumDrive drive;

    public CustomMecanumDrive() {
        DuneStrider robot = DuneStrider.get();
        drive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);
    }

    public void drive(float forward, float strafe, float rotate) {
        drive.driveRobotCentric(strafe, forward, rotate);
    }

    @Override
    public void periodic() {
        Telemetry t = DuneStrider.get().telemetry;
        t.addLine("========DRIVE TRAIN========");
    }
}
