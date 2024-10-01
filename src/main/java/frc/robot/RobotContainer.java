// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SensorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final CommandXboxController joystick = new CommandXboxController(0); 
  public final Drivetrain drivetrain = TunerConstants.DriveTrain; 
  public final RobotState robotState = RobotState.getInstance();
  //public final ClimberJoint climberJoint = new ClimberJoint();
  //public final ElevatorJoint elevatorJoint = new ElevatorJoint();
  public final ElevatorRollers elevatorRollers = new ElevatorRollers();
  public final IntakeJoint intakeJoint = new IntakeJoint();
  public final IntakeRollers intakeRollers = new IntakeRollers();
  public final ShooterJoint shooterJoint = new ShooterJoint();
  public final ShooterRollers shooterRollers = new ShooterRollers();
  public final YSplitRollers ySplitRollers = new YSplitRollers();

  private final LaserCanSensor lc1 = new LaserCanSensor(SensorConstants.ID_LC1);
  private final LaserCanSensor lc2 = new LaserCanSensor(SensorConstants.ID_LC2);
  private final DigitalInput bb1 = new DigitalInput(SensorConstants.PORT_BB1);


   private final Trigger LC1 = new Trigger(() -> lc1.isClose());
  private final Trigger LC2 = new Trigger(() -> lc2.isClose());
  private final Trigger noteStored = new Trigger(() -> (lc1.isClose() || lc2.isClose()));
  private final Trigger noteAmp = new Trigger(() -> !bb1.get());
  /* 
  private final Trigger readyToShoot = new Trigger(() -> (shooterRollers.getState() != ShooterRollers.State.OFF) && shooterRollers.atGoal() && shooterJoint.atGoal());
  private final Trigger readyToAmp = new Trigger(() -> (elevatorJoint.getState() != ElevatorJoint.State.SCORE) && elevatorJoint.atGoal());
  private final Trigger scoreTrigger = joystick.rightTrigger(); */

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.run(
        () -> drivetrain.setControllerInput(-joystick.getLeftY(), -joystick.getLeftX(), -joystick.getRightX())));    

    drivetrain.registerTelemetry(logger::telemeterize);

    //joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

     //Intake
    joystick.leftTrigger()
        .whileTrue(robotState.setTargetCommand(RobotState.TARGET.NOTE)
            .alongWith(intakeJoint.setStateCommand(IntakeJoint.State.INTAKE)
                .alongWith(Commands.waitUntil(intakeJoint::atGoal)
                    .andThen(intakeRollers.setStateCommand(IntakeRollers.State.INTAKE)
                        .alongWith(ySplitRollers.setStateCommand(YSplitRollers.State.INTAKE))))
                .until(LC2)));
                

    //Subwoofer
    joystick.a()
        .whileTrue(robotState.setTargetCommand(RobotState.TARGET.SUBWOOFER)
            .alongWith(shooterRollers.setStateCommand(ShooterRollers.State.SUBWOOFER))
            .alongWith(shooterJoint.setStateCommand(ShooterJoint.State.SUBWOOFER)));
    
    // AMP
    //TODO break scoring out better
/*     joystick.b()
        .whileTrue(robotState.setTargetCommand(RobotState.TARGET.AMP)
            .alongWith(elevatorJoint.setStateCommand(ElevatorJoint.State.STOW)
                .alongWith(Commands.waitUntil(elevatorJoint::atGoal)
                    .andThen(ySplitRollers.setStateCommand(YSplitRollers.State.AMP)
                        .alongWith(elevatorRollers.setStateCommand(ElevatorRollers.State.INTAKE))))
                .until(noteAmp)
                .andThen(elevatorJoint.setStateCommand(ElevatorJoint.State.SCORE))
                .alongWith(Commands.waitUntil(scoreTrigger.and(readyToAmp))
                    .andThen(elevatorRollers.setStateCommand(ElevatorRollers.State.SCORE))))); */
     // Feed
    joystick.y()
        .whileTrue(robotState.setTargetCommand(RobotState.TARGET.FEED)
            .alongWith(ySplitRollers.setStateCommand(YSplitRollers.State.AMP))
        );
            //.alongWith(shooterJoint.setStateCommand(ShooterJoint.State.DYNAMIC)));

    joystick.back().whileTrue(elevatorRollers.setStateCommand(ElevatorRollers.State.SCORE));

/*     //Speaker
    joystick.x().whileTrue(robotState.setTargetCommand(RobotState.TARGET.SPEAKER)
            .alongWith(shooterRollers.setStateCommand(ShooterRollers.State.SPEAKER))
            .alongWith(shooterJoint.setStateCommand(ShooterJoint.State.DYNAMIC)));

    //Climb
    joystick.start()
        .whileTrue(shooterJoint.setStateCommand(ShooterJoint.State.CLIMBCLEARANCE)
            .alongWith(Commands.waitUntil(shooterJoint::atGoal)
                .andThen(climberJoint.setStateCommand(ClimberJoint.State.CLIMB))));  */

  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
