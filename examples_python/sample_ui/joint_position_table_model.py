from robot_state import RobotState
from common import *
import math
from PySide6.QtCore import Qt, QAbstractTableModel, QModelIndex
from PySide6.QtWidgets import QApplication, QTableView, QDoubleSpinBox, QStyledItemDelegate, QVBoxLayout, QWidget, \
    QPushButton, QStyleOptionSpinBox, QItemDelegate
from PySide6.QtGui import QStandardItemModel, QStandardItem, QPen, QColor
from PySide6.QtWidgets import (QApplication, QMainWindow, QTableView, QSpinBox,
                               QStyledItemDelegate, QVBoxLayout, QWidget, QAbstractItemView, QHeaderView)
from PySide6.QtCore import Qt, QAbstractTableModel, QModelIndex


class JointPositionTableModel(QAbstractTableModel):
    def __init__(self, target_index):
        super(JointPositionTableModel, self).__init__()
        self.headers = ["Name", "T.pos", "Name", "T.pos"]
        self.target_index = target_index
        self.target_position = [0] * len(self.target_index)

    def get_idx(self, index):
        return math.floor(index.column() / 2) + index.row() * 2

    def data(self, index, role):
        idx = self.get_idx(index)
        if idx >= len(self.target_index):
            return
        if len(RobotState.joint_states) == 0:
            return

        if role == Qt.DisplayRole or role == Qt.EditRole:
            if index.column() == 0 or index.column() == 2:
                return RobotState.joint_states[self.target_index[idx]]['name']
            if index.column() == 1 or index.column() == 3:
                return f"{self.target_position[idx]:.2f}"

    def rowCount(self, index):
        return math.floor((len(self.target_index) + 1) / 2)

    def columnCount(self, index):
        return 4

    def setData(self, index, value, role=Qt.EditRole):
        if role == Qt.EditRole:
            idx = self.get_idx(index)
            self.target_position[idx] = value
            self.dataChanged.emit(index, index, [Qt.DisplayRole, Qt.EditRole])
            return True
        return False

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return self.headers[section]
            if orientation == Qt.Vertical:
                return section + 1

    def flags(self, index):
        if index.column() == 0 or index.column() == 2:
            return Qt.ItemIsSelectable | Qt.ItemIsEnabled
        if index.column() == 1 or index.column() == 3:
            return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable

    def emitAllDataChanged(self):
        top_left = self.index(0, 0)
        bottom_right = self.index(self.rowCount(QModelIndex()) - 1, self.columnCount(QModelIndex()) - 1)
        self.dataChanged.emit(top_left, bottom_right, [Qt.DisplayRole])


class SpinBoxDelegate(QStyledItemDelegate):
    def createEditor(self, parent, option, index):
        spin_box = QDoubleSpinBox(parent)
        # spin_box.setMinimum(-3.14)
        # spin_box.setMaximum(3.14)
        # spin_box.setSingleStep(0.01)
        spin_box.setMinimum(-180)
        spin_box.setMaximum(180)
        spin_box.setSingleStep(0.01)
        return spin_box

    def setEditorData(self, editor, index):
        value = index.model().data(index, Qt.EditRole)
        editor.setValue(float(value))

    def setModelData(self, editor, model, index):
        editor.interpretText()
        value = editor.value()
        model.setData(index, value, Qt.EditRole)
