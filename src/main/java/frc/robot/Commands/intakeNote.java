package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.YSplitRollers;
import frc.robot.subsystems.IntakeJoint;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.Util.LaserCanSensor;



public class intakeNote extends Command{
    IntakeJoint m_intakeJoint;
    IntakeRollers m_intakeRollers;
    YSplitRollers m_ySplitRollers;
    boolean m_isDone;

    public intakeNote(IntakeJoint intakeJoint, IntakeRollers intakeRollers, YSplitRollers ySplitRollers) {
        m_intakeJoint = intakeJoint;
        m_intakeRollers = intakeRollers;
        m_ySplitRollers = ySplitRollers;
        addRequirements(intakeJoint, intakeRollers, ySplitRollers);
    }

    @Override
    public void initialize() {
        System.out.println("intakeNote Starting");
        m_isDone = false;
    }

   
}
