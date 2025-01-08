package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor armR = hardwareMap.dcMotor.get("armR");
        DcMotor armL = hardwareMap.dcMotor.get("armL");
        DcMotor armExtnd = hardwareMap.dcMotor.get("armExtnd");
        Servo Claw = hardwareMap.get(Servo.class, "Claw");
        Servo ClawL = hardwareMap.get(Servo.class, "ClawL");
        Servo ClawR = hardwareMap.get(Servo.class, "ClawR");

        // Reverse motors if needed (this depends on your robot configuration)
        // frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set arm motors to BRAKE mode to hold position when no joystick input is given
        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Mecanum drive controls (simplified version)
            double y = -gamepad1.left_stick_y;  // Forward/Backward movement
            double x = (gamepad1.left_trigger + -gamepad1.right_trigger) * 1.1;  // Strafing movement
            double rx = gamepad1.left_stick_x;  // Rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Arm control logic
            double armPowerL;
            double armPowerR;

            // Debugging joystick value to see if it's being read correctly
            telemetry.addData("Joystick Value", gamepad2.left_stick_y);

            if (gamepad2.left_stick_y < 0) {
                // Full speed upwards
                armPowerL = -gamepad2.left_stick_y / 0.25; // Positive value (up)
                armPowerR = -gamepad2.left_stick_y / 0.25; // Positive value (up)
                telemetry.addData("Arm Direction", "Up");
            } else if (gamepad2.left_stick_y > 0) {
                // Resistance when going down (scale down the power)
                armPowerL = -gamepad2.left_stick_y * 0.5; // Negative value (down), but slower
                armPowerR = -gamepad2.left_stick_y * 0.5; // Same for the other arm motor
                telemetry.addData("Arm Direction", "Down with Resistance");

            } else {
                // When joystick is centered, hold position
                armPowerL = 0;  // No movement when joystick is centered
                armPowerR = 0;  // No movement when joystick is centered
                telemetry.addData("Arm Direction", "Idle (Hold)");
            }

            telemetry.addData("ArmL", armL);
            telemetry.addData("ArmR", armR);

            double Up;
             Up = gamepad2.right_stick_y;
            armExtnd.setPower(Up);

            if (Up > 0.5) {
                armPowerR = -gamepad2.left_stick_y * 0.8;
                armPowerL = -gamepad2.left_stick_y * 0.8;
            }

            // Apply power to arm motors
            armL.setPower(armPowerL);
            armR.setPower(armPowerR);

            // Claw control
            if (gamepad2.a) {
                Claw.setPosition(1);  // Open Claw
            } else if (gamepad2.x) {
                Claw.setPosition(0);  // Close Claw
            }

            // ClawL and ClawR control (independent)
            if (gamepad2.y) {
                ClawL.setPosition(0.9);  // Open ClawL
                ClawR.setPosition(0.7);  // Open ClawR
            }

            if (gamepad2.b) {
                //default behavior
                ClawL.setPosition(0.1);  // Close ClawL
                ClawR.setPosition(.3);  // Close ClawR
            }

            // Telemetry to display arm positions and power values for debugging
            telemetry.addData("ArmL Power", armPowerL);
            telemetry.addData("ArmR Power", armPowerR);
            telemetry.addData("ArmL Position", armL.getCurrentPosition());
            telemetry.addData("ArmR Position", armR.getCurrentPosition());
            telemetry.addData("ClawL Position", ClawL.getPosition());
            telemetry.addData("ClawR Position", ClawR.getPosition());
            telemetry.update();
        }
    }
}
