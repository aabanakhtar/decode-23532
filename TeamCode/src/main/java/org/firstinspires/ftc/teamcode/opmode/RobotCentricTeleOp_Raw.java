package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is a "raw" FTC SDK TeleOp for testing robot-centric mecanum driving.
 * It does NOT use the DuneStrider class or any command-based structure.
 * It initializes motors directly and controls them in a single loop.
 *
 * CONTROLS:
 * - Left Stick Y:  Drive Forward/Backward
 * - Left Stick X:  Strafe Left/Right
 * - Right Stick X: Rotate Left/Right
 */
@TeleOp(name = "Robot Centric TeleOp (Raw SDK)", group = "Diagnostics")
public class RobotCentricTeleOp_Raw extends LinearOpMode {

    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing motors for raw TeleOp...");

        // Initialize motors from hardware map
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // *** IMPORTANT: SET MOTOR DIRECTIONS ***
        // Reversing the left side motors as requested.
        // The right side remains as FORWARD.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to brake when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialization Complete. Ready for start.");
        telemetry.addLine("Controls: Left Stick (Translate), Right Stick (Rotate)");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // --- Gamepad Input ---
            // The FTC SDK gamepad inputs are reversed for the Y-axis.
            // We multiply by -1 to correct this.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            // --- Mecanum Drive Calculation ---
            // This is the core mecanum drive logic.
            double leftFrontPower = forward + strafe + rotation;
            double rightFrontPower = forward - strafe - rotation;
            double leftBackPower = forward - strafe + rotation;
            double rightBackPower = forward + strafe - rotation;

            // --- Normalize Motor Powers ---
            // Find the maximum power to scale all powers down if any are > 1.0
            double maxPower = 1.0;
            maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
            maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            maxPower = Math.max(maxPower, Math.abs(rightBackPower));

            // Apply the scaled power to the motors
            leftFront.setPower(leftFrontPower / maxPower);
            rightFront.setPower(rightFrontPower / maxPower);
            leftBack.setPower(leftBackPower / maxPower);
            rightBack.setPower(rightBackPower / maxPower);

            // --- Telemetry ---
            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Rotation", "%.2f", rotation);
            telemetry.addLine();
            telemetry.addData("LF Power", "%.2f", leftFrontPower);
            telemetry.addData("RF Power", "%.2f", rightFrontPower);
            telemetry.addData("LB Power", "%.2f", leftBackPower);
            telemetry.addData("RB Power", "%.2f", rightBackPower);
            telemetry.update();
        }

        // Stop all motors when the OpMode ends
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}

