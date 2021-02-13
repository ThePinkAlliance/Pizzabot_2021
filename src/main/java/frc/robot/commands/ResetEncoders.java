package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class ResetEncoders extends CommandBase {

    private final DriveTrain m_drive;
    

    public ResetEncoders(DriveTrain driveTrain) {
        m_drive = driveTrain;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
