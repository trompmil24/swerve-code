// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.autonomous;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController m_controller = new XboxController(0);
  JoystickButton buttonA = new JoystickButton(m_controller, 1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   Consumer<ChassisSpeeds> speed = new Consumer<ChassisSpeeds>() {

    @Override
    public void accept(ChassisSpeeds arg0) {
            // TODO Auto-generated method stub
            
    }
};
Consumer<SwerveModuleState[]> state = new Consumer<SwerveModuleState[]>() {

  @Override
  public void accept(SwerveModuleState[] arg0) {
          // TODO Auto-generated method stub
          
  }
  
};
Consumer<Pose2d> poseConsumer = new Consumer<Pose2d>() {

  @Override
  public void accept(Pose2d arg0) {
          // TODO Auto-generated method stub
          
  }
  
};
Supplier<Pose2d> position = () -> new Pose2d();
SwerveDriveKinematics m_kinematics = m_drivetrainSubsystem.getKinematics();

  PathPlannerTrajectory practicePath = PathPlanner.loadPath("practice", new PathConstraints(10, 2));

  HashMap<String, Command> eventMap = new HashMap<>();
  //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  //eventMap.put("intakeDown", new IntakeDown());
  //ArrayList<PathPlannerTrajectory> pathGroup = new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("practice", new PathConstraints(3, 2)));
  
  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      position, // Pose2d supplier
      poseConsumer, // Pose2d consumer, used to reset odometry at the beginning of auto
      m_kinematics, // SwerveDriveKinematics
      new PIDConstants(.25, 0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(2.8, 0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
      state, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_drivetrainSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
  );
  
  Command fullAuto = autoBuilder.fullAuto(practicePath);



        PIDController pidX = new PIDController(.25, 0, 0);
        PIDController pidY = new PIDController(.25, 0, 0);
        PIDController pidRotation = new PIDController(2.8, 0, 0);
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
          new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               //this.resetOdometry(traj.getInitialHolonomicPose());
           }
           
         }),
         new PPSwerveControllerCommand(traj, position, pidX, pidY, pidRotation, speed, isFirstPath, m_drivetrainSubsystem)
     );
 }


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return followTrajectoryCommand(practicePath, false);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}