package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.device.SwyftRanger;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

public class SensorStack extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();

    public SensorStack() {
    }

    @Override
    public void periodic() {
        robot.flightRecorder.addData("Ranger 1", robot.ranger0.getDistance());
        robot.flightRecorder.addData("Ranger 2", robot.ranger1.getDistance());
    }
}
