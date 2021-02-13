package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.*;


public class PizzaDash {

    public void putEncoder(DriveTrain drive) {
        if (drive != null) {
            SmartDashboard.putNumber("Encoder DriveTrain FR", drive.getFrontRightDistance());
            SmartDashboard.putNumber("Encoder DriveTrain FL", drive.getFrontLeftDistance());
            SmartDashboard.putNumber("Encoder DriveTrain BR", drive.getBackRightDistance());
            SmartDashboard.putNumber("Encoder DriveTrain BL", drive.getBackLeftDistance());
        }
    }
}