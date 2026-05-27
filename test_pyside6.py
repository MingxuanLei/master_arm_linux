import sys
from PySide6.QtWidgets import (QApplication, QWidget, QLabel, QLineEdit,
                               QRadioButton, QPushButton, QVBoxLayout,
                               QHBoxLayout, QMessageBox)

class Calculator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("100以内加减法 (PySide6)")
        self.resize(300, 200)

        layout = QVBoxLayout()

        self.label_a = QLabel("数字1 (0-100):")
        layout.addWidget(self.label_a)
        self.entry_a = QLineEdit()
        layout.addWidget(self.entry_a)

        self.label_b = QLabel("数字2 (0-100):")
        layout.addWidget(self.label_b)
        self.entry_b = QLineEdit()
        layout.addWidget(self.entry_b)

        # 运算符单选
        op_layout = QHBoxLayout()
        self.radio_add = QRadioButton("加法 (+)")
        self.radio_sub = QRadioButton("减法 (-)")
        self.radio_add.setChecked(True)
        op_layout.addWidget(self.radio_add)
        op_layout.addWidget(self.radio_sub)
        layout.addLayout(op_layout)

        self.calc_btn = QPushButton("计算")
        self.calc_btn.clicked.connect(self.calculate)
        layout.addWidget(self.calc_btn)

        self.result_label = QLabel("结果：")
        layout.addWidget(self.result_label)

        self.setLayout(layout)

    def calculate(self):
        try:
            a = int(self.entry_a.text())
            b = int(self.entry_b.text())
        except ValueError:
            QMessageBox.critical(self, "错误", "请输入整数！")
            return

        if not (0 <= a <= 100 and 0 <= b <= 100):
            QMessageBox.warning(self, "超限", "数字必须在 0~100 之间！")
            return

        if self.radio_add.isChecked():
            result = a + b
            if result > 100:
                QMessageBox.warning(self, "超限", "加法结果超过 100！")
                return
        else:
            result = a - b
            if result < 0:
                QMessageBox.warning(self, "超限", "减法结果小于 0！")
                return

        self.result_label.setText(f"结果：{result}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Calculator()
    window.show()
    sys.exit(app.exec())