// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;
import java.util.function.Supplier;
import java.util.function.BiConsumer;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Trajectory Following Info
 * The paths are generated without any MalformedSplineExceptions
 * The robot will continue moving for the estimated time length of the path.
 * Ex. PathWeaver estimates path will take 5.6 seconds, then robot will move for 5.6 seconds in auto before shutting down.
 * Robot is following the Pathweaver trajectory, but will only get partway along the path before quitting.
 * So path generation is fine, robot is just moving too slowly along the path?
 * On Carpet: Romi ended at X=3.26,Y=6.55; Z=3.33,Y=6.53
 * On Tuned Wood Floor: Romi ended at X=3.37;Y=5.89;X=3.41,6.27
 * Romi should end at X=7.31,Y=4.61
 * Changing kPDriveVel from 0 to 3.7 did not help speed, but seemed to make drive less smooth
 * FRC Characterization was done when encoders were in inches.
 * Next, need to recharacterize now that encoders are in meters.
 * /


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Don't print Joystick connection warnings to dashboard
    DriverStation.getInstance().silenceJoystickConnectionWarning(true);
    
  }

   /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand() {
    Trajectory exampleTrajectory = getTrajectorySample();

    Supplier<Pose2d> pose = m_drivetrain::getPoseRadians;
    Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds = m_drivetrain::getWheelSpeeds;
    BiConsumer<Double, Double> tankDriveVolts = m_drivetrain::tankDriveVolts;

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory, 
      pose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      wheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      tankDriveVolts,
      m_drivetrain);


    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_drivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    //m_chooser.setDefaultOption("Ramsete Trajectory", generateRamseteCommand());
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    
    SmartDashboard.putData(m_chooser);
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Trajectory trajectory = getTrajectoryFigureEightFromPathWeaver();
    Trajectory trajectory = getTrajectoryFromPathweaver("SmoothCurve");

    Command trajectoryCommand = getAutoRamseteCommand(trajectory);
    return trajectoryCommand;
    //return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> m_controller.getRawAxis(2));
  }


public double getGyroAngleZ() {
	return m_drivetrain.getGyroAngleZ();
}

public Pose2d getPose() {
  return m_drivetrain.getPoseRadians();
}

public Command getAutoRamseteCommand(Trajectory trajectory){
  double trajectoryTime = trajectory.getTotalTimeSeconds();
  System.out.print(trajectoryTime);
  System.out.println(" seconds to complete trajectory");
  System.out.println(trajectory.getInitialPose());
  if (trajectoryTime != 0){
    System.out.println(trajectory.sample(trajectoryTime-.01));
  }
  
  RamseteCommand ramseteCommand = new RamseteCommand(
    trajectory,
    m_drivetrain::getPoseRadians,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                              DriveConstants.kvVoltSecondsPerMeter,
                              DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    m_drivetrain::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_drivetrain::tankDriveVolts,
    m_drivetrain
  );

  // Reset odometry to the starting pose of the trajectory.
  m_drivetrain.resetOdometry(trajectory.getInitialPose());

  InstantCommand resetOdometry = new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose()));
   
  // Run path following command, then stop at the end.
  return resetOdometry.andThen(ramseteCommand).andThen(() -> m_drivetrain.tankDriveVolts(0, 0)).andThen(() -> System.out.println("Finished driving auto trajectory path"));
}

public Trajectory getTrajectoryFromPathweaver(String trajectoryName){
  String trajectoryJSON = "paths/" + trajectoryName + ".wpilib.json";
  Trajectory trajectory = new Trajectory();
  try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }

  System.out.println("Starting Path");
  DriverStation.reportWarning("Start Trajectory Path "+ trajectoryName,false);

  return trajectory;
}

public Trajectory getTrajectorySlalom(){
  return getTrajectoryFromPathweaver("Slalom");
}

public Trajectory getTrajectoryYoyo(){
  var autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                 DriveConstants.kvVoltSecondsPerMeter, 
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);
  // Create config for trajectory.
  TrajectoryConfig config =
  new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
    // Apply the voltage constraint
    .addConstraint(autoVoltageConstraint);
    
    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.)
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(5, 5, new Rotation2d(0)),
        List.of(
            new Translation2d(8, 5),
            new Translation2d(8, 8),
            new Translation2d(5, 8),
            new Translation2d(5, 5),
            new Translation2d(2, 2)
        ),
        new Pose2d(2, 2, new Rotation2d(0)), config);
    return exampleTrajectory;
}

public Trajectory getTrajectorySample(){
  var autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                 DriveConstants.kvVoltSecondsPerMeter, 
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);
  // Create config for trajectory.
  TrajectoryConfig config =
  new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
    // Apply the voltage constraint
    .addConstraint(autoVoltageConstraint);
    
    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.)
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(2, 0.25),
            new Translation2d(2, -0.25),
            new Translation2d(2, 0.25),
            new Translation2d(2, -0.25)
        ),
        new Pose2d(8.0, 0, new Rotation2d(0)), config);
    return exampleTrajectory;
}

public Trajectory getTrajectoryFigureEightFromPathWeaver(){
  String trajectoryJSON = "paths/FigureEight.wpilib.json";
  Trajectory trajectory = new Trajectory();
  try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }
  return trajectory;
}

public Trajectory getTrajectoryFigureEight(){
  System.out.println("Generating Trajectory Figure Eight");
  var autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                 DriveConstants.kvVoltSecondsPerMeter, 
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);
  // Create config for trajectory.
  TrajectoryConfig config =
  new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
    // Apply the voltage constraint
    .addConstraint(autoVoltageConstraint);
    
    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.)
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 1),
            new Translation2d(1, -1),
            new Translation2d(1, -1),
            new Translation2d(1, 1),
            new Translation2d(-1, 1),
            new Translation2d(-1, -1),
            new Translation2d(-1, -1),
            new Translation2d(-1, 1)
        ),
        /**List.of(
            new Translation2d(1, 1),
            new Translation2d(2, 0),
            new Translation2d(3, -1),
            new Translation2d(4, 0),
            new Translation2d(3, 1),
            new Translation2d(2, 0),
            new Translation2d(1, -1),
            new Translation2d(0, 0)
        ),*/
        new Pose2d(0, 0, new Rotation2d(0)), config);
    return exampleTrajectory;
}
}

