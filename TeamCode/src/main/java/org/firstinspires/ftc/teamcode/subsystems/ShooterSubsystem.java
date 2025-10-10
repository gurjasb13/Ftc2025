package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;
    public static PController controller;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");

        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goTo(double setPoint) {
        double output = controller.calculate(shooterMotor1.getCurrentPosition(), setPoint);
        controller.setSetPoint(setPoint);

    }
}
