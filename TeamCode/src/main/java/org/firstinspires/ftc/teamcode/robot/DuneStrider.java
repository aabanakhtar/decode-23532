package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Hubs;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.Arrays;
import java.util.List;

public class DuneStrider {
    public enum Alliance {
        RED, BLUE
    }

    private static final DuneStrider inst = new DuneStrider();

    // alliance settings
    public static Alliance alliance = Alliance.RED;

    // hardware (besides dt, managed by pedro)
    public MotorEx shooterLeft, shooterRight, shooterTurret;
    public MotorEx intakeTubing;
    public CRServoEx leftTransferWheel, rightTransferWheel;

    // hubs
    public List<LynxModule> lynxModules;
    public HardwareMap hardwareMap;

    // subsystems for FTCLib
    public Hubs hubs;
    public MecanumDrive drive = null;
    public Shooter shooter;
    public Intake intake;
    public Turret turret;

    public static DuneStrider get() {
        return inst;
    }

    public MultipleTelemetry telemetry;

    public DuneStrider() {}

    public DuneStrider init(Pose pose, HardwareMap map, Telemetry t) {
        CommandScheduler.getInstance().reset();
        hardwareMap = map;
        lynxModules = map.getAll(LynxModule.class);

        // Shooter motors
        shooterLeft = new MotorEx(map, "shooterLeft").setCachingTolerance(0.001);
        shooterRight = new MotorEx(map, "shooterRight").setCachingTolerance(0.001);
        // Reverse one shooter motor so they spin the same way i think
        shooterRight.setInverted(true);

        // Turret motor
        shooterTurret = new MotorEx(map, "shooterTurret").setCachingTolerance(0.001);
        shooterTurret.stopAndResetEncoder();

        // Intake motor
        intakeTubing = new MotorEx(map, "intake").setCachingTolerance(0.001);
        intakeTubing.setInverted(true);

        // Apply common run modes
        Arrays.asList(shooterLeft, shooterRight, shooterTurret, intakeTubing)
                .forEach(x -> {
                    x.setRunMode(Motor.RunMode.RawPower);
                });

        // Transfer wheels
        leftTransferWheel = new CRServoEx(map, "leftTransferWheel")
                .setRunMode(CRServoEx.RunMode.RawPower);
        rightTransferWheel = new CRServoEx(map, "rightTransferWheel")
                .setRunMode(CRServoEx.RunMode.RawPower);
        leftTransferWheel.setInverted(true);
        telemetry = new MultipleTelemetry(t, PanelsTelemetry.INSTANCE.getFtcTelemetry(), FtcDashboard.getInstance().getTelemetry());

        // subsystem init
        hubs = new Hubs();
        drive = new MecanumDrive(map, pose);
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        return inst;
    }


    public void endLoop() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
