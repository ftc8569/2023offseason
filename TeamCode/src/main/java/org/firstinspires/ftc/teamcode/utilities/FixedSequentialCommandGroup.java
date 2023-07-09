package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class FixedSequentialCommandGroup extends SequentialCommandGroup {

    private boolean m_isFinished = false;

    public FixedSequentialCommandGroup(Command... commands) {
        super(commands);
    }

    @Override
    public boolean isFinished() {
        m_isFinished = m_isFinished || super.isFinished();
        return m_isFinished;
    }


}
