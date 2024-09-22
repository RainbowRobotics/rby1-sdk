from robot_state import RobotState
from common import *
from PySide6.QtCore import Qt, QAbstractTableModel, QModelIndex
from PySide6.QtGui import QBrush, QColor
import numpy as np

class JointStatesTableModel(QAbstractTableModel):
    def __init__(self):
        super(JointStatesTableModel, self).__init__()
        self.headers = ["Name", "F", "R", "I", "J", "C", "B", "Pos (deg)", "Vel (deg/s)", "Cur (Amp)",
                        "Tq (Nm)"]

    def data(self, index, role):
        if role == Qt.DisplayRole:
            if index.column() == 0:
                return RobotState.joint_states[index.row()]['name']
            if index.column() == 7:
                return f"{np.rad2deg(RobotState.joint_states[index.row()]['position']):2.3f}"
            if index.column() == 8:
                return f"{np.rad2deg(RobotState.joint_states[index.row()]['velocity']):2.3f}"
            if index.column() == 9:
                return f"{RobotState.joint_states[index.row()]['current']:2.3f}"
            if index.column() == 10:
                return f"{RobotState.joint_states[index.row()]['torque']:2.3f}"
        elif role == Qt.BackgroundRole:
            motor_type = RobotState.joint_states[index.row()]['motor_type']
            motor_state = RobotState.joint_states[index.row()]['motor_state']

            if index.column() == 1:
                return QBrush(QColor(GREEN_CODE if RobotState.joint_states[index.row()]['fet'] else RED_CODE))
            if index.column() == 2:
                return QBrush(QColor(GREEN_CODE if RobotState.joint_states[index.row()]['run'] else RED_CODE))
            if index.column() == 3:
                return QBrush(QColor(GREEN_CODE if RobotState.joint_states[index.row()]['init'] else RED_CODE))
            if index.column() == 4:
                return QBrush(QColor(RED_CODE if motor_type == 1 and ((motor_state >> 8) & 1) == 1 else GREEN_CODE))
            if index.column() == 5:
                return QBrush(QColor(RED_CODE if motor_type == 1 and ((motor_state >> 9) & 1) == 1 else GREEN_CODE))
            if index.column() == 6:
                return QBrush(QColor(RED_CODE if motor_type == 1 and ((motor_state >> 10) & 1) == 1 else GREEN_CODE))
        elif role == Qt.ToolTipRole:
            if index.column() == 0:
                return RobotState.joint_states[index.row()]['name']

    def rowCount(self, index):
        return len(RobotState.joint_states)

    def columnCount(self, index):
        return len(self.headers)

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return self.headers[section]
            if orientation == Qt.Vertical:
                return section + 1

    def emitAllDataChanged(self):
        top_left = self.index(0, 0)
        bottom_right = self.index(self.rowCount(QModelIndex()) - 1, self.columnCount(QModelIndex()) - 1)
        self.dataChanged.emit(top_left, bottom_right, [Qt.DisplayRole])
