package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.device.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.device.SwyftRanger;
import org.firstinspires.ftc.teamcode.subsystem.ArduCam;
import org.firstinspires.ftc.teamcode.subsystem.Hubs;
import org.firstinspires.ftc.teamcode.subsystem.MegaTagRelocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.SensorStack;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.utilities.BasicFilter;
import org.firstinspires.ftc.teamcode.utilities.RunningAverageFilter;

import java.util.Arrays;
import java.util.List;

@Config
public class DuneStrider {
    private static final DuneStrider inst = new DuneStrider();
    public final static double IDEAL_VOLTAGE = 12.5;
    public static double TURRET_ENCODER_OFFSET = 21.5;

    public enum Mode {
        AUTO,
        TELEOP
    }

    public enum Alliance {
        RED, BLUE
    }

    // alliance settings
    public static Alliance alliance = Alliance.BLUE;
    public static Mode mode = Mode.AUTO;

    // hardware (besides dt, managed by pedro)
    public MotorEx shooterLeft, shooterRight, shooterTurret;
    public MotorEx intakeTubing;
    public ServoEx latchServo;

    // sensors
    public Limelight3A limelight;
    public VoltageSensor batterySensor;
    private final BasicFilter batteryFilter = new RunningAverageFilter(3);
    private final BasicFilter intakeFilter = new RunningAverageFilter(4);
    public AbsoluteAnalogEncoder analogEncoder;

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
    public ArduCam cam;
    public MegaTagRelocalizer eyes;

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

        //limelight.pipelineSwitch(2);
        //limelight.start();
    }

    public DuneStrider init(Mode mode, Pose pose, HardwareMap map, Telemetry t) {
        CommandScheduler.getInstance().reset();
        batteryFilter.reset();
        Turret.offset_angle = 0;

        DuneStrider.mode = mode;
        hardwareMap = map;
        lynxModules = map.getAll(LynxModule.class);
        lynxModules.forEach(x -> {
            x.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        });

        analogEncoder = new AbsoluteAnalogEncoder(hardwareMap, "abs", TURRET_ENCODER_OFFSET);

        // sensors
        batterySensor = hardwareMap.getAll(VoltageSensor.class)
                .iterator()
                .next();

        // Shooter motors
        shooterLeft = new MotorEx(map, "shooterLeft").setCachingTolerance(0.000001);
        shooterRight = new MotorEx(map, "shooterRight").setCachingTolerance(0.000001);
        // Reverse one shooter motor so they spin the same way i think
        shooterLeft.setInverted(true);

        // Turret motor
        shooterTurret = new MotorEx(map, "shooterTurret", Motor.GoBILDA.RPM_312).setCachingTolerance(0.0001);
        shooterTurret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Intake motor
        intakeTubing = new MotorEx(map, "intake").setCachingTolerance(0.1);
        intakeTubing.setInverted(false);

        latchServo = new ServoEx(map, "latch").setCachingTolerance(0.1);

        // Apply common run modes
        Arrays.asList(shooterLeft, shooterRight, intakeTubing)
                .forEach(x -> {
                    x.setRunMode(Motor.RunMode.RawPower);
                });

        flightRecorder = new MultipleTelemetry(t);

        // subsystem init
        hubs = new Hubs();
        drive = new MecanumDrive(map, pose);
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        sensors = new SensorStack();
        eyes = new MegaTagRelocalizer();
        cam = new ArduCam(hardwareMap.get(WebcamName.class, "cam"));
        reset();
        return inst;
    }

    public double getVoltage() {
        return lastMeasuredVoltage;
    }

    public double getIntakeVoltage() {
        return intakeFilter.getFilteredOutput();
    }

    public boolean has3Balls() {
        return intakeFilter.getFilteredOutput() > Intake.INTAKE_3_BALLS;
    }

    public double getVoltageFeedforwardConstant() {
        return batteryFilter.getFilteredOutput();
    }

    public void endLoop() {
        flightRecorder.addData(">>>>ALLIANCE", alliance.toString());
        // sensor update
        flightRecorder.addData(">>>>BATTERY STATE", lastMeasuredVoltage);
        flightRecorder.addData(">>>>INTAKE VOLTAGE", getIntakeVoltage());
        flightRecorder.addData("Has 3 balls?", has3Balls());

        lastMeasuredVoltage = batterySensor.getVoltage();
        double safeVoltage = Math.max(lastMeasuredVoltage, 9.0);
        double intakeVoltage = intakeTubing.getCurrent(CurrentUnit.AMPS);
        batteryFilter.updateValue(IDEAL_VOLTAGE / safeVoltage);
        intakeFilter.updateValue(intakeVoltage);

        CommandScheduler.getInstance().run();
        flightRecorder.update();

    }
}
