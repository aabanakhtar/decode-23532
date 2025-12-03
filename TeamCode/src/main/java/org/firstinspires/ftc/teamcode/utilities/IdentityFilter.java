package org.firstinspires.ftc.teamcode.utilities;

public class IdentityFilter implements BasicFilter {
    double lastValue = 0.0;

    @Override
    public void updateValue(double value) {
        lastValue = value;
        return;
    }

    @Override
    public double getFilteredOutput() {
        return lastValue;
    }

    @Override
    public void reset() {
        lastValue = 0.0;
    }


}
