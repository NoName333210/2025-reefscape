// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.limelight.Odometry;

import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

  // controller declarations
  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController m_coDriverController = new CommandXboxController(1);

  // subsystem declarations
  public static final Swerve m_swerve = new Swerve();
  public static final CTREConfigs ctreConfigs = new CTREConfigs();


  public static final Odometry m_odometry = new Odometry(Constants.Swerve.swerveKinematics, m_swerve.getRotation2d(), m_swerve.getModulePositions(), m_swerve.getPose());




  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /**
     * this is an example of how to assign button :
     * m_driverController.a().onTrue(ALGAE_INTAKE_AUTO); (so clean i know)
     */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
