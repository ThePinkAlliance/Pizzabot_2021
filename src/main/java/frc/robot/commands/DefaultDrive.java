package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class DefaultDrive extends CommandBase {

    private final DriveTrain m_drive;
    private final DoubleSupplier leftSpeed;
    private final DoubleSupplier rightSpeed;

    public DefaultDrive(DriveTrain driveTrain, DoubleSupplier left, DoubleSupplier right) {
        m_drive = driveTrain;
        leftSpeed = left;
        rightSpeed = right;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
    }
    
}
