package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@Configurable
public class Turret extends SubsystemBase {
    private final DuneStrider robot = DuneStrider.get();

    public static bool tuning = false;
    public static double targetPower = 0.0;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static PIDFController controller;

    public Turret() {

    }

    @Override
    public void periodic() {
        robot.shooterTurret.set(targetPower);
    }

    public void calculateInterpolatedVelocity() {

    }
}
