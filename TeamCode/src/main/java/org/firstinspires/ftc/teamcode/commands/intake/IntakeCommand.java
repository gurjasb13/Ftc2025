package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    private final Gamepad gamepad;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, Gamepad gamepad) {
        this.intakeSubsystem = intakeSubsystem;
        this.gamepad = gamepad;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        double power = gamepad.b ? 1 : 0;
        intakeSubsystem.setPower(power);
    }

    public void end(){
        intakeSubsystem.stop();
    }
}
