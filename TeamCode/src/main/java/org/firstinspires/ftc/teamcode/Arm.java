package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm { @TeleOp(name="Motor test", group="Test")
public class MotorSample extends OpMode {

    DcMotor arm = null;

    @Override
    public void init() {
        arm = hardwareMap.get(DcMotor.class, "arm");
    }

    @Override
    public void loop() {
        arm.setPower(gamepad2.right_stick_y);
        telemetry.addData("position", arm.getCurrentPosition());
        telemetry.update();
    }
}
}
