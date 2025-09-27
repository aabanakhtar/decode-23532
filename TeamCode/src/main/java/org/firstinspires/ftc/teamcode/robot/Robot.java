package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.subsystem.BulkCachingHubs;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecDrive;
import org.firstinspires.ftc.teamcode.subsystem.TurretShooter;

public class Robot {
    private static final Robot inst = new Robot();

    public BulkCachingHubs hubs;

    public MecDrive drive;

    public Intake intake;

    public TurretShooter shooter;

    public static Robot get() {
        return inst;
    }

    Robot() {

    }

}
