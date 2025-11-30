package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.device.SwyftRanger;
import org.firstinspires.ftc.teamcode.subsystem.Hubs;
import org.firstinspires.ftc.teamcode.subsystem.HugeEyes;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.SensorStack;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.Arrays;
import java.util.List;

public class DuneStrider {
    private static final DuneStrider inst = new DuneStrider();
    public final static double IDEAL_VOLTAGE = 13.0;

    public enum Alliance {
        RED, BLUE
    }

    // alliance settings
    public static Alliance alliance = Alliance.BLUE;

    // hardware (besides dt, managed by pedro)
    public MotorEx shooterLeft, shooterRight, shooterTurret;
    public MotorEx intakeTubing;
    public ServoEx latchServo;

    // sensors
    public Limelight3A limelight;
    public VoltageSensor batterySensor;

    public SwyftRanger ranger0;
    public SwyftRanger ranger1;

    // hubs
    public List<LynxModule> lynxModules;
    public HardwareMap hardwareMap;

    // subsystems for FTCLib
    public Hubs hubs;
    public MecanumDrive drive;
    public SensorStack sensors;
    public Shooter shooter;
    public Intake intake;
    public Turret turret;

    // for writing logs
    public MultipleTelemetry flightRecorder;

    // for feedforward
    private double lastMeasuredVoltage = IDEAL_VOLTAGE;

    public static DuneStrider get() {
        return inst;
    }

    public DuneStrider() {}

    public void reset() {
        intake.setMode(Intake.Mode.OFF);
        turret.setMode(Turret.Mode.HOMING);
        shooter.setIdle();
        shooter.setPower(0);
    }

    public DuneStrider init(Pose pose, HardwareMap map, Telemetry t) {
        CommandScheduler.getInstance().reset();
        hardwareMap = map;
        lynxModules = map.getAll(LynxModule.class);
        ranger0 = new SwyftRanger(hardwareMap, "ranger0");
        ranger1 = new SwyftRanger(hardwareMap, "ranger1");


        // sensors
        batterySensor = hardwareMap.getAll(VoltageSensor.class)
                .iterator()
                .next();

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
        intakeTubing.setInverted(false);

        latchServo = new ServoEx(map, "latch").setCachingTolerance(0.001);

        // Apply common run modes
        Arrays.asList(shooterLeft, shooterRight, shooterTurret, intakeTubing)
                .forEach(x -> {
                    x.setRunMode(Motor.RunMode.RawPower);
                });

        // Transfer wheels
        flightRecorder = new MultipleTelemetry(t, PanelsTelemetry.INSTANCE.getFtcTelemetry(), FtcDashboard.getInstance().getTelemetry());

        // subsystem init
        hubs = new Hubs();
        drive = new MecanumDrive(map, pose);
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        sensors = new SensorStack();
        reset();
        return inst;
    }


    public double getVoltageFeedforwardConstant() {
        // 11.0 is just we don't place unnecessary strain if somehow the battery drops to something like 4v.
        double safeVoltage = Math.max(lastMeasuredVoltage, 9.0);
        return IDEAL_VOLTAGE / safeVoltage;
    }

    public void endLoop() {
        flightRecorder.addData("ALLIANCE", alliance.toString());
        // sensor update
        lastMeasuredVoltage = batterySensor.getVoltage();
        flightRecorder.addData("BATTERY STATE", lastMeasuredVoltage);

        CommandScheduler.getInstance().run();
        flightRecorder.update();

    }
}
