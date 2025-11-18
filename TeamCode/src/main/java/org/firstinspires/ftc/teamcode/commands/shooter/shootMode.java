package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class shootMode extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    Gate gate;
    private final Gamepad gamepad;

    public shootMode(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, Gate gate, Gamepad gamepad) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.gate = gate;
        this.gamepad = gamepad;
        addRequirements(shooterSubsystem, intakeSubsystem, gate);
    }

    @Override
    public void execute() {
        if (gamepad.right_bumper) {
            intakeSubsystem.setPower(1);
            shooterSubsystem.setPower(1);
        } else {
            intakeSubsystem.setPower(0);
            shooterSubsystem.setPower(0);
        }

        if (gamepad.y) {
            gate.setPosition(0);
        }else{
            gate.setPosition(0.4);
        }
    }

    @Override
    public boolean isFinished() {
        return !gamepad.right_bumper && !gamepad.y;
    }
}
