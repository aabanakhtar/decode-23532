package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

import java.util.List;

public class Hubs extends SubsystemBase {
    private final List<LynxModule> modules;
    private final ElapsedTime timer;
    double dt = 0;

    public Hubs() {
        modules = DuneStrider.get().lynxModules;
        timer = new ElapsedTime();
        clearCache();
    }

    public void clearCache() {
        modules.forEach(LynxModule::clearBulkCache);

        dt = timer.seconds();
        if (dt > 0) {
            // send info to ftc dash and phone
            DuneStrider.get().flightRecorder.addLine("=====SYSTEM=======");
            DuneStrider.get().flightRecorder.addData("Loop Time, (HZ)", 1.0 / dt);
        }

        timer.reset();
    }

    @Override
    public void periodic() {
        clearCache();
    }

    public double getDeltaTime() {
        return dt;
    }
}
