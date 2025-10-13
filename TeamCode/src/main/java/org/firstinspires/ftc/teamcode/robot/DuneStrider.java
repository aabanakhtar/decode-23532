package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Hubs;
import org.firstinspires.ftc.teamcode.subsystem.CustomMecanumDrive;
import java.util.Arrays;
import java.util.List;

public class DuneStrider {
    public enum Alliance {
        RED, BLUE
    }

    private static final DuneStrider inst = new DuneStrider();

    // alliance settings
    public static Alliance alliance = Alliance.RED;

    // hardware
    public MotorEx leftFront, leftBack, rightFront, rightBack;
    public MotorEx shooterLeft, shooterRight, shooterTurret;
    public MotorEx intake;
    public CRServoEx leftTransferWheel, rightTransferWheel;

    // hubs
    public List<LynxModule> lynxModules;

    // subsystems for ftclib
    public Hubs hubs;
    public CustomMecanumDrive drive;

    public static DuneStrider get() {
        return inst;
    }

    public MultipleTelemetry telemetry;

    public DuneStrider() {}

    public DuneStrider init(HardwareMap map, Telemetry t) {
        // Drivetrain motors
        leftFront = new MotorEx(map, "leftFront").setCachingTolerance(0.001);
        rightFront = new MotorEx(map, "rightFront").setCachingTolerance(0.001);
        leftBack = new MotorEx(map, "leftBack").setCachingTolerance(0.001);
        rightBack = new MotorEx(map, "rightBack").setCachingTolerance(0.001);

        rightFront.setInverted(true);
        rightBack.setInverted(true);

        // Shooter motors
        shooterLeft = new MotorEx(map, "shooterLeft").setCachingTolerance(0.001);
        shooterRight = new MotorEx(map, "shooterRight").setCachingTolerance(0.001);

        // Reverse one shooter motor so they spin the same way i think
        shooterRight.setInverted(true);

        // Turret motor
        shooterTurret = new MotorEx(map, "shooterTurret").setCachingTolerance(0.001);
        shooterTurret.stopAndResetEncoder();

        // Intake motor
        intake = new MotorEx(map, "intake").setCachingTolerance(0.001);

        // Apply common run modes
        Arrays.asList(leftFront, rightFront, leftBack, rightBack,
                        shooterLeft, shooterRight, shooterTurret, intake)
                .forEach(x -> {
                    x.setRunMode(Motor.RunMode.RawPower);
                    x.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                });

        // Transfer wheels
        leftTransferWheel = new CRServoEx(map, "leftTransferWheel")
                .setRunMode(CRServoEx.RunMode.RawPower);
        rightTransferWheel = new CRServoEx(map, "rightTransferWheel")
                .setRunMode(CRServoEx.RunMode.RawPower);
        rightTransferWheel.setInverted(true);

        leftTransferWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightTransferWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        // lynx modules and bulk caching setup
        lynxModules = map.getAll(LynxModule.class);
        lynxModules.forEach(x -> {
            x.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        });
        lynxModules.forEach(LynxModule::clearBulkCache);

        telemetry = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());

        // subsystem init
        drive = new CustomMecanumDrive();
        hubs = new Hubs();
        return inst;
    }


    public void endLoop() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
