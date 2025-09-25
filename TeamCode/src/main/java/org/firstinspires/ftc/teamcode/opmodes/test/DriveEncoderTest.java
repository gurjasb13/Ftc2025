package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DriveEncoderTest extends OpMode {
    private DcMotor par1;
    private DcMotor par2;
    private DcMotor perp;

    @Override
    public void init() {
        par1 = hardwareMap.get(DcMotor.class, "par1");
        par2 = hardwareMap.get(DcMotor.class, "par2");
        perp = hardwareMap.get(DcMotor.class, "perp");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("par1", par1.getCurrentPosition());
        telemetry.addData("par2", par2.getCurrentPosition());
        telemetry.addData("perp", perp.getCurrentPosition());
        telemetry.update();
    }
}
