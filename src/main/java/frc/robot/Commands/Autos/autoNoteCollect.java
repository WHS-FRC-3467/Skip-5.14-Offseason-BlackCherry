package frc.robot.Commands.Autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeJoint;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.YSplitRollers;

//NO VISION IN YET
public class autoNoteCollect {
    Drivetrain m_drivetrain;
    IntakeJoint m_intakeJoint;
    IntakeRollers m_intakeRollers;
    YSplitRollers m_ySplitRollers;
    //Limelight m_limelight;

    SwerveRequest.FieldCentricFacingAngle m_head;

    public autoNoteCollect(Drivetrain drivetrain, IntakeJoint intakeJoint, IntakeRollers intakeRollers, YSplitRollers ySplitRollers) {
        m_drivetrain = drivetrain;
        m_intakeJoint = intakeJoint;
        m_intakeRollers = intakeRollers;
        m_ySplitRollers = ySplitRollers;

        addCommands(new driveToNote(m_drivetrain).withTimeout(2.50));
        addCommands(new intakeNote(m_intakeJoint, m_intakeRollers));
    };

}
