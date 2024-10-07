// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.InjectableValues;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SensorConstants;
import frc.robot.Util.LaserCanSensor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
public class RobotContainer {

	private final CommandXboxController joystick = new CommandXboxController(0);
	public final Drivetrain drivetrain = TunerConstants.DriveTrain;
	public final RobotState robotState = RobotState.getInstance();
	public final ClimberJoint climberJoint = new ClimberJoint();
	public final ElevatorJoint elevatorJoint = new ElevatorJoint();
	public final ElevatorRollers elevatorRollers = new ElevatorRollers();
	public final IntakeJoint intakeJoint = new IntakeJoint();
	public final IntakeRollers intakeRollers = new IntakeRollers();
	public final ShooterJoint shooterJoint = new ShooterJoint();
	public final ShooterRollers shooterRollers = new ShooterRollers();
	public final YSplitRollers ySplitRollers = new YSplitRollers();

	private final LaserCanSensor lc1 = new LaserCanSensor(SensorConstants.ID_LC1);
	private final LaserCanSensor lc2 = new LaserCanSensor(SensorConstants.ID_LC2);

	private final Debouncer ampDebouncer = new Debouncer(.25, DebounceType.kBoth);
	private final DigitalInput bb1 = new DigitalInput(SensorConstants.PORT_BB1);

	private final Trigger LC1 = new Trigger(() -> lc1.isClose());
	private final Trigger LC2 = new Trigger(() -> lc2.isClose());
    private final Trigger BB1 = new Trigger(() -> !bb1.get());
	private final Trigger noteStored = new Trigger(() -> (lc1.isClose() || lc2.isClose()));
	private final Trigger noteAmp = new Trigger(() -> ampDebouncer.calculate(!bb1.get()));

	private final Trigger readyToShoot = new Trigger(() -> (shooterRollers.getState() != ShooterRollers.State.OFF)
			&& shooterRollers.atGoal() && shooterJoint.atGoal());
	private final Trigger readyToAmp = new Trigger(
			() -> (elevatorJoint.getState() == ElevatorJoint.State.SCORE) && elevatorJoint.atGoal());
	private final Trigger readyToClimb = new Trigger(
			() -> (shooterJoint.getState() == ShooterJoint.State.CLIMBCLEARANCE) && shooterJoint.atGoal())
			.and(readyToAmp);
	private final Trigger atClimb = new Trigger(
	        () -> (climberJoint.getState() == ClimberJoint.State.CLIMB) && climberJoint.atGoal());

	private SendableChooser<Command> autoChooser;

	private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

	private void configureBindings() {
		drivetrain.setDefaultCommand(drivetrain.run(() -> drivetrain.setControllerInput(-joystick.getLeftY(),
				-joystick.getLeftX(), -joystick.getRightX())));

		drivetrain.registerTelemetry(logger::telemeterize);

		// joystick.povUp().onTrue(drivetrain.runOnce(() ->
		// drivetrain.seedFieldRelative()));

		// Intake
		joystick.leftTrigger().whileTrue(Commands.parallel(
				robotState.setTargetCommand(RobotState.TARGET.NOTE),
				intakeJoint.setStateCommand(IntakeJoint.State.INTAKE),
				Commands.waitUntil(intakeJoint::atGoal)
						.andThen(Commands.deadline(
								Commands.waitUntil(LC1),
								intakeRollers.setStateCommand(
										IntakeRollers.State.INTAKE),
								ySplitRollers.setStateCommand(
										YSplitRollers.State.INTAKE)))
						.andThen(Commands.deadline(
								Commands.waitUntil(LC2),
								ySplitRollers.setStateCommand(
										YSplitRollers.State.SLOWINTAKE)))));

		// Subwoofer
		joystick.a().whileTrue(
				Commands.parallel(
						robotState.setTargetCommand(RobotState.TARGET.SUBWOOFER),
						shooterRollers.setStateCommand(ShooterRollers.State.SUBWOOFER),
						shooterJoint.setStateCommand(ShooterJoint.State.SUBWOOFER)));

		// Amp
		joystick.b().whileTrue(
				Commands.parallel(
						robotState.setTargetCommand(RobotState.TARGET.AMP),
						elevatorJoint.setStateCommand(ElevatorJoint.State.STOW),
						Commands.waitUntil(elevatorJoint::atGoal)
								.andThen(Commands.deadline(
										Commands.waitUntil(LC2.negate()),
										intakeRollers.setStateCommand(IntakeRollers.State.EJECT),
										ySplitRollers.setStateCommand(YSplitRollers.State.SHUFFLE)))
								.andThen(Commands.deadline(
										Commands.waitUntil(noteAmp),
										ySplitRollers.setStateCommand(YSplitRollers.State.AMP),
										intakeRollers.setStateCommand(IntakeRollers.State.INTAKE),
										elevatorRollers.setStateCommand(
												ElevatorRollers.State.INTAKE))))
						.withInterruptBehavior(InterruptionBehavior.kCancelSelf));

		// Feed
		joystick.y().whileTrue(
				Commands.parallel(
						robotState.setTargetCommand(RobotState.TARGET.FEED),
						// drivetrain.setStateCommand(Drivetrain.State.HEADING),
						shooterRollers.setStateCommand(ShooterRollers.State.FEED),
						shooterJoint.setStateCommand(ShooterJoint.State.DYNAMIC)));

		// Speaker
		joystick.x().whileTrue(
				Commands.parallel(
						robotState.setTargetCommand(RobotState.TARGET.SPEAKER),
						// drivetrain.setStateCommand(Drivetrain.State.HEADING),
						shooterRollers.setStateCommand(ShooterRollers.State.SPEAKER),
						shooterJoint.setStateCommand(ShooterJoint.State.DYNAMIC)));

		// Climb
		joystick.start().whileTrue(
				Commands.parallel(
						shooterJoint.setStateCommand(ShooterJoint.State.CLIMBCLEARANCE),
						elevatorJoint.setStateCommand(ElevatorJoint.State.SCORE),
						Commands.waitUntil(readyToClimb)
								.andThen(climberJoint.setStateCommand(
										ClimberJoint.State.CLIMB)),
						Commands.waitUntil(atClimb)
								.andThen(elevatorRollers.setStateCommand(ElevatorRollers.State.SCORE))));

		// Score
		joystick.rightTrigger().whileTrue(
				// Commands.either(
				Commands.either(
						Commands.deadline(
								Commands.waitUntil(noteAmp.negate()),
								elevatorJoint.setStateCommand(ElevatorJoint.State.SCORE),
								Commands.waitUntil(readyToAmp)
										.andThen(elevatorRollers.setStateCommand(ElevatorRollers.State.SCORE))),
						Commands.deadline(
								Commands.waitUntil(noteStored.negate()),
								Commands.waitUntil(readyToShoot)
										.andThen(ySplitRollers.setStateCommand(YSplitRollers.State.SHOOTER))),
						noteAmp));


		joystick.povLeft().whileTrue(
				Commands.parallel(
						intakeJoint.setStateCommand(IntakeJoint.State.HOMING).until(() -> intakeJoint.hasHomed),
						elevatorJoint.setStateCommand(ElevatorJoint.State.HOMING).until(() -> elevatorJoint.hasHomed),
						Commands.deadline(
								Commands.waitUntil(() -> climberJoint.hasHomed),
								shooterJoint.setStateCommand(ShooterJoint.State.CLIMBCLEARANCE),
								Commands.waitUntil(() -> shooterJoint.getState() == ShooterJoint.State.CLIMBCLEARANCE && shooterJoint.atGoal())
										.andThen(climberJoint.setStateCommand(ClimberJoint.State.HOMING)))));

		joystick.povRight().whileTrue(
				Commands.parallel(
						ySplitRollers.setStateCommand(YSplitRollers.State.REVSHOOTER),
						intakeRollers.setStateCommand(IntakeRollers.State.EJECT)));


	}

	private void registerNamedCommands() {
		NamedCommands.registerCommand("ShootSpeaker", null);
	}

	private void configureDebugCommands() {
		SmartDashboard.putData("YSplit Eject Shooter",Commands.parallel(ySplitRollers.setStateCommand(YSplitRollers.State.REVSHOOTER)));
		SmartDashboard.putData("YSplit Eject Amp",Commands.parallel(ySplitRollers.setStateCommand(YSplitRollers.State.REVAMP)));
		SmartDashboard.putData("Intake Eject",Commands.parallel(intakeRollers.setStateCommand(IntakeRollers.State.EJECT)));
		SmartDashboard.putData("Intake Deploy",Commands.parallel(intakeJoint.setStateCommand(IntakeJoint.State.INTAKE)));
		SmartDashboard.putData("Shooter Eject",Commands.parallel(shooterRollers.setStateCommand(ShooterRollers.State.REVERSE)));
		SmartDashboard.putData("Elevator Stow",Commands.parallel(elevatorJoint.setStateCommand(ElevatorJoint.State.STOW)));
		SmartDashboard.putData("Elevator Score",Commands.parallel(elevatorJoint.setStateCommand(ElevatorJoint.State.SCORE)));
		SmartDashboard.putData("Elevator Homing",Commands.parallel(elevatorJoint.setStateCommand(ElevatorJoint.State.HOMING)));
        SmartDashboard.putData("Intake Homing",Commands.parallel(intakeJoint.setStateCommand(IntakeJoint.State.HOMING)));
        SmartDashboard.putData("Climber Homing",Commands.parallel(climberJoint.setStateCommand(ClimberJoint.State.HOMING)));
		SmartDashboard.putData("Shooter Tuning Angle",Commands.parallel(shooterJoint.setStateCommand(ShooterJoint.State.TUNING)));
		SmartDashboard.putData("Shooter Climber Clearance",Commands.parallel(shooterJoint.setStateCommand(ShooterJoint.State.CLIMBCLEARANCE)));
		SmartDashboard.putData("Shooter Roller Speaker",Commands.parallel(shooterRollers.setStateCommand(ShooterRollers.State.SPEAKER)));
        SmartDashboard.putData("Shooter Roller Sub",Commands.parallel(shooterRollers.setStateCommand(ShooterRollers.State.SUBWOOFER)));
	}

    public void displaySystemInfo() {
        SmartDashboard.putBoolean("Beam Break 1", BB1.getAsBoolean());
		SmartDashboard.putBoolean("Lasercan 1", LC1.getAsBoolean());
		SmartDashboard.putBoolean("Lasercan 2", LC2.getAsBoolean());
		SmartDashboard.putBoolean("readyToScore",readyToShoot.getAsBoolean());
		SmartDashboard.putBoolean("readyToAmp",readyToAmp.getAsBoolean());
		SmartDashboard.putBoolean("readyToClimb",readyToClimb.getAsBoolean());
        SmartDashboard.putBoolean("Note in YSplit", noteStored.getAsBoolean());
        SmartDashboard.putBoolean("Note in Amp", noteAmp.getAsBoolean());
    }

	public RobotContainer() {
		configureBindings();
		configureDebugCommands();
		registerNamedCommands();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
