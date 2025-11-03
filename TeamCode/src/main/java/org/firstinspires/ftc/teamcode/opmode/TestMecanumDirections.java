package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is a diagnostic OpMode to test that each motor is correctly wired and
 * configured. Each of the four main gamepad buttons (Y, X, B, A) corresponds
 * to one motor.
 *
 * CONTROLS:
 * - Y Button:        Powers the Left Front motor
 * - X Button:        Powers the Left Back motor
 * - B Button:        Powers the Right Front motor
 * - A Button:        Powers the Right Back motor
 *
 * PROCEDURE:
 * 1. Lift the robot off the ground so the wheels can spin freely.
 * 2. Press 'Y'. The front-left wheel should spin. If not, check your wiring and config name.
 * 3. Press 'X'. The back-left wheel should spin.
 * 4. Press 'B'. The front-right wheel should spin.
 * 5. Press 'A'. The back-right wheel should spin.
 *
 * All motors should spin in the FORWARD direction when tested. The direction
 * reversal for driving will be handled in your actual drive code, not here.
 */
@TeleOp(name = "Test Individual Motors", group = "Diagnostics")
@Disabled
public class TestMecanumDirections extends LinearOpMode {

    // Declare motor objects
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Define a constant for the test power
    private final double TEST_POWER = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing motors for individual testing...");
        telemetry.update();

        // --- MOTOR INITIALIZATION ---
        // Map the motors to their names in the robot configuration
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        // For this test, we want to see the raw motor direction.
        // Set all motors to FORWARD to see their natural spin direction.
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to brake to prevent coasting after a button is released.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialization Complete.");
        telemetry.addLine("Y = Left Front | X = Left Back");
        telemetry.addLine("B = Right Front | A = Right Back");
        telemetry.addLine("!!! LIFT ROBOT OFF GROUND BEFORE STARTING !!!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // Set all motor powers to zero at the start of the loop
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            // Check each button and apply power to the corresponding motor
            if (gamepad1.y) {
                leftFront.setPower(TEST_POWER);
                telemetry.addLine("Testing LEFT FRONT motor");
            } else if (gamepad1.x) {
                leftBack.setPower(TEST_POWER);
                telemetry.addLine("Testing LEFT BACK motor");
            } else if (gamepad1.b) {
                rightFront.setPower(TEST_POWER);
                telemetry.addLine("Testing RIGHT FRONT motor");
            } else if (gamepad1.a) {
                rightBack.setPower(TEST_POWER);
                telemetry.addLine("Testing RIGHT BACK motor");
            } else {
                telemetry.addLine("No motor selected. Press Y, X, B, or A.");
            }

            // Update telemetry
            telemetry.update();
        }

        // Stop all motors on exit
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
