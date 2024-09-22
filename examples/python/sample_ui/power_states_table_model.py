from robot_state import RobotState
from common import *
from PySide6.QtCore import Qt, QAbstractTableModel, QModelIndex
from PySide6.QtGui import QBrush, QColor


class PowerStatesTableModel(QAbstractTableModel):
    def __init__(self):
        super(PowerStatesTableModel, self).__init__()
        self.headers = ["Name", "Voltage (V)", "State"]

    def data(self, index, role):
        if role == Qt.DisplayRole:
            if index.column() == 0:
                return RobotState.power_states[index.row()]['name']
            if index.column() == 1:
                return f"{RobotState.power_states[index.row()]['voltage']:2.3f}"
        elif role == Qt.BackgroundRole:
            if index.column() == 2:
                return QBrush(QColor(GREEN_CODE if RobotState.power_states[index.row()]['state'] else RED_CODE))
        elif role == Qt.ToolTipRole:
            if index.column() == 0:
                return RobotState.power_states[index.row()]['name']

    def rowCount(self, index):
        return len(RobotState.power_states)

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
