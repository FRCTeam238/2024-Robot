// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TrajectoryDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import java.nio.file.FileSystem;
import java.util.List;
import monologue.Logged;
import monologue.Monologue;
import org.frc238.lib.autonomous.AutonomousModesReader;
import org.frc238.lib.autonomous.DataFileAutonomousModeDataSource;
import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

public class Robot extends TimedRobot implements Logged {
  private Command m_autonomousCommand;

  private AutonomousModesReader amodeReader;
  private List<String> autoNames;
  private SendableChooser<String> autoChooser;
  private String lastSelectedAuto;

  public static Constants.RobotState state = Constants.RobotState.INTAKE;

  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator;// = new Elevator();
  public static Intake intake = new Intake();
  public static Feeder feeder;// = new Feeder();
  public static Pivot pivot;// = new Pivot();
  public static Shooter shooter;// = new Shooter();
  public static OI oi = new OI();

  @Override
  public void robotInit() {
   

    Monologue.setupMonologue(this, "Robot", false, false);
    SignalLogger.start();
    DataLogManager.start();
    URCL.start();
    amodeReader = new AutonomousModesReader(new DataFileAutonomousModeDataSource(Filesystem.getDeployDirectory() + "/amode238.txt"));
    autoChooser = new SendableChooser<>();
    autoNames = amodeReader.GetAutoNames();
    for (String name : autoNames) {
      autoChooser.setDefaultOption(name, name);
    }
    SmartDashboard.putData(autoChooser);
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Monologue.updateAll();

    if (lastSelectedAuto != autoChooser.getSelected()) {
        m_autonomousCommand =
      amodeReader.getAutonomousMode(autoChooser.getSelected());
    }
    
    lastSelectedAuto = autoChooser.getSelected();
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Monologue.setFileOnly(DriverStation.isFMSAttached());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    // m_autonomousCommand = new TrajectoryDriveCommand("NewPath", true, 1);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }



  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static Constants.RobotState getState() {
    return state;
  }
}
