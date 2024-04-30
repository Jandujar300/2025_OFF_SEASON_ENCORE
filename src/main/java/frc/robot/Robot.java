// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;


public class Robot extends LoggedRobot {
  LimelightHelpers.Results lastResult;
  boolean buildAuto = true;


  @Override

  
  public void robotInit() {
    new RobotContainer();
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();  
    lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults; 

  }

  
  @Override
  public void disabledInit() {
     CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

}