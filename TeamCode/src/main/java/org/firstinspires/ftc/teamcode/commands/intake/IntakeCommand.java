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
        if (gamepad.right_stick_y == 1){
            intakeSubsystem.setPower(-1);
        } else if (gamepad.right_stick_y == -1){
            intakeSubsystem.setPower(1);
        } else{
            intakeSubsystem.setPower(0);
        }
    }

    public void end(){
        intakeSubsystem.stop();
    }
}
