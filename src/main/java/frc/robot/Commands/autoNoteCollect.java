package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeJoint;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.YSplitRollers;

//NO VISION IN YET
public class autoNoteCollect extends ParallelRaceGroup{
    Drivetrain m_drivetrain;
    IntakeJoint m_intakeJoint;
    IntakeRollers m_intakeRollers;
    YSplitRollers m_ySplitRollers;
    //Limelight m_limelight;
    

    SwerveRequest.FieldCentricFacingAngle m_head;

    public autoNoteCollect(Drivetrain drivetrain, IntakeJoint intakeJoint, IntakeRollers intakeRollers, YSplitRollers ySplitRollers, SwerveRequest.FieldCentricFacingAngle head) {
        m_drivetrain = drivetrain;
        m_intakeJoint = intakeJoint;
        m_intakeRollers = intakeRollers;
        m_ySplitRollers = ySplitRollers;
        m_head = head;

       addCommands(new driveToNote(m_drivetrain, m_head));
       addCommands(new intakeNote(m_intakeJoint, m_intakeRollers, m_ySplitRollers));
    }

    private void addCommands(intakeNote intakeNote) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addCommands'");
    }

    private void addCommands(driveToNote driveToNote) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addCommands'");
    };

}
