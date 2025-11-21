package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.drivebase.ChatAlign;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;


public class DriveEncoderTest extends OpMode {
    private DcMotor par1;
    private DcMotor par2;
    private DcMotor perp;

    private DrivebaseSubsystem drivebaseSubsystem;
    public ChatAlign chatAlign;

    @Override
    public void init() {
        par1 = hardwareMap.get(DcMotor.class, "lb");
        par2 = hardwareMap.get(DcMotor.class, "rf");
        perp = hardwareMap.get(DcMotor.class, "rb");

        par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        perp.setDirection(DcMotor.Direction.REVERSE);

        drivebaseSubsystem=new DrivebaseSubsystem(hardwareMap);
        chatAlign=new ChatAlign(drivebaseSubsystem, gamepad1, telemetry, hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("par1", par1.getCurrentPosition());
        telemetry.addData("par2", par2.getCurrentPosition());
        telemetry.addData("perp", perp.getCurrentPosition());
        telemetry.update();

        chatAlign.execute();
    }
}
