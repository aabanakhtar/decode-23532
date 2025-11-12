package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@TeleOp(name = "⚙️ Pre-Match Configure robot")
public class ConfigureRobot extends OpMode {
    private Prompter prompter;

    @Override
    public void init() {
        prompter = new Prompter(this);
        prompter.prompt("alliance", new OptionPrompt<>("Alliance select", DuneStrider.Alliance.RED, DuneStrider.Alliance.BLUE));
        prompter.prompt("rows", new ValuePrompt("Rows select", 0.0, 3.0, 0.0, 1.0));

        prompter.onComplete(() -> {
            DuneStrider.alliance = prompter.get("alliance");
            Autonomous.nRows = prompter.get("rows");
            telemetry.addLine("Done configuring!");
        });

    }

    @Override
    public void loop() {
        prompter.run();
    }
}
