package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    public double   scale(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return (toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow) + toLow;
    }
    @Override

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor"); // port 0
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor"); // port 1
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor"); // port 2
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor"); // port 3
        DcMotor armR = hardwareMap.dcMotor.get("armR"); // expansion hub -- port 0
        DcMotor Spin = hardwareMap.dcMotor.get("armExtnd"); // expansion hub -- port 1
        DcMotor armL = hardwareMap.dcMotor.get("armL"); // expansion hub -- port 2
        Servo Claw = hardwareMap.get(Servo.class, "Claw");
        Servo ClawL = hardwareMap.get(Servo.class, "ClawL");
        Servo ClawR = hardwareMap.get(Servo.class, "ClawR");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot move   s backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

       // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
  //         backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
          // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double[] xpmult = {
                    0.0, 1.0,
                    0.0, 1.0
            };
            double[] xnmult = {
                    1.0, 0.0,
                    1.0, 0.0
            };
            double[] ypmult = {
                    1.0, -1.0,
                    0.0, 0.0
            };
            double[] ynmult = {
                    0.0, 0.0,
                    1.0, -1.0
            };


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = (gamepad1.left_trigger + -gamepad1.right_trigger) * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;
            double rorx = gamepad1.right_stick_x;
            double rory = gamepad1.right_stick_y;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;





              {

                if(gamepad2.a){;
                    Claw.setPosition(1);
                }
                if(gamepad2.x){
                  Claw.setPosition(0);
              }




                  telemetry.addData("Claw", Claw.getPosition());


                  telemetry.addData("ClawL", ClawL.getPosition());
                  telemetry.addData("ClawR", ClawR.getPosition());


            }

            if (gamepad2.y){ClawL.setPosition(0);}
           else {ClawL.setPosition(1);}

            if (gamepad2.y){ClawR.setPosition(1);}
            else {ClawR.setPosition(0);}

            double Rotaion;

            double Spin1 = gamepad2.right_stick_y;

            Spin1 = Math.min(Spin1, 1);
            Spin1 = Math.max(Spin1, -1);

            Spin.setPower(Spin1);




                double armPower = gamepad2.left_stick_y;
                double armPower1 = gamepad2.left_stick_y;

                // Clamp armPower within -0.8 to 0.8
            armPower = Math.max(-1, Math.min(armPower, 1));
            armPower = Math.max(-1, Math.min(armPower, 1));

       

//                double currentPower = arm1.getPower();
//                currentPower = Arm2.getPower();
//                armPower = currentPower + (targetPower - curr entPower) * 0.1;

                armL.setPower(armPower);
                armR.setPower(armPower1);
                int targetPositionL = armL.getCurrentPosition();
                int targetPositionR = armR.getCurrentPosition();


                telemetry.addData("arm power", armPower);
                telemetry.addData("arm power1", armPower1);
                telemetry.addData("arm powerR", targetPositionL);
                telemetry.addData("arm powerL", targetPositionR);
                telemetry.update();


                if (rorx >= 0) {
                    frontLeftPower = frontLeftPower + xpmult[0] * rorx / Math.min(Math.abs(frontLeftPower) + Math.abs(xpmult[0]) * Math.abs(rorx), 1);
                    frontRightPower = frontRightPower + xpmult[1] * rorx / Math.min(Math.abs(frontRightPower) + Math.abs(xpmult[1]) * Math.abs(rorx), 1);
                    backLeftPower = backLeftPower + xpmult[2] * rorx / Math.min(Math.abs(backLeftPower) + Math.abs(xpmult[2]) * Math.abs(rorx), 1);
                    backRightPower = backRightPower + xpmult[3] * rorx / Math.min(Math.abs(backRightPower) + Math.abs(xpmult[3]) * Math.abs(rorx), 1);
                } else {
                    frontLeftPower = frontLeftPower - xpmult[0] * rorx / Math.min(Math.abs(frontLeftPower) + Math.abs(xnmult[0]) * Math.abs(rorx), 1);
                    frontRightPower = frontRightPower - xpmult[1] * rorx / Math.min(Math.abs(frontRightPower) + Math.abs(xnmult[1]) * Math.abs(rorx), 1);
                    backLeftPower = backLeftPower - xpmult[2] * rorx / Math.min(Math.abs(backLeftPower) + Math.abs(xnmult[2]) * Math.abs(rorx), 1);
                    backRightPower = backRightPower - xpmult[3] * rorx / Math.min(Math.abs(backRightPower) + Math.abs(xnmult[3]) * Math.abs(rorx), 1);
                }

                if (rory >= 0) {
                    frontLeftPower = frontLeftPower + xpmult[0] * rorx / Math.min(Math.abs(frontLeftPower) + Math.abs(ypmult[0]) * Math.abs(rory), 1);
                    frontRightPower = frontRightPower + xpmult[1] * rorx / Math.min(Math.abs(frontRightPower) + Math.abs(ypmult[1]) * Math.abs(rory), 1);
                    backLeftPower = backLeftPower + xpmult[2] * rorx / Math.min(Math.abs(backLeftPower) + Math.abs(ypmult[2]) * Math.abs(rory), 1);
                    backRightPower = backRightPower + xpmult[3] * rorx / Math.min(Math.abs(backRightPower) + Math.abs(ypmult[3]) * Math.abs(rory), 1);
                } else {
                    frontLeftPower = frontLeftPower - xpmult[0] * rorx / Math.min(Math.abs(frontLeftPower) + Math.abs(ynmult[0]) * Math.abs(rory), 1);
                    frontRightPower = frontRightPower - xpmult[1] * rorx / Math.min(Math.abs(frontRightPower) + Math.abs(ynmult[1]) * Math.abs(rory), 1);
                    backLeftPower = backLeftPower - xpmult[2] * rorx / Math.min(Math.abs(backLeftPower) + Math.abs(ynmult[2]) * Math.abs(rory), 1);
                    backRightPower = backRightPower - xpmult[3] * rorx / Math.min(Math.abs(backRightPower) + Math.abs(ynmult[3]) * Math.abs(rory), 1);
                }

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                telemetry.update();

            }
        } }