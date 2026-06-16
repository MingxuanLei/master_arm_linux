"""
GUIyemian_teleoperation.py

Linux 下弱双向遥操作 + 遥操作记录 + 示教回放的外层 GUI 控制界面。

说明：
    本文件不重复实现底层遥操作算法，而是直接调用同目录下的
    teleoperation_shuangxiangshijiao_linux.py。

    GUI 通过 QProcess 启动该脚本，并向脚本标准输入发送命令：
        1 -> 遥操作模式
        2 -> 遥操作记录模式
        3 -> 示教回放模式
        0 -> 安全退出

推荐目录结构：
    GUIyemian_teleoperation.py
    teleoperation_shuangxiangshijiao_linux.py
    GUIyemian_7motors_linux.py
    USBCANFD_gai.py
    USBCANFD_DEMO.py
    libusbcanfd.so
    DMMotor.py
    Robot.py
    TreeStruct.py

运行：
    python3 GUIyemian_teleoperation.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QProcess, QProcessEnvironment, Qt, QTimer
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSplitter,
    QDoubleSpinBox,
    QVBoxLayout,
    QWidget,
)


CMD_EXIT = "0"
CMD_TELEOP = "1"
CMD_RECORD = "2"
CMD_REPLAY = "3"

MODE_TO_ARG = {
    CMD_TELEOP: "teleop",
    CMD_RECORD: "record",
    CMD_REPLAY: "replay",
}

MODE_TO_NAME = {
    CMD_TELEOP: "遥操作模式",
    CMD_RECORD: "遥操作记录模式",
    CMD_REPLAY: "示教回放模式",
}


class TeleoperationProcessController:
    """QProcess 封装：负责启动、停止和向后端脚本发送模式命令。"""

    def __init__(self, parent: QMainWindow):
        self.parent = parent
        self.process: Optional[QProcess] = None
        self.current_script: Optional[Path] = None

    def is_running(self) -> bool:
        return self.process is not None and self.process.state() != QProcess.NotRunning

    def start(self, script_path: Path, initial_cmd: str, extra_args: list[str]) -> bool:
        if self.is_running():
            self.parent.append_log("[WARN] 后端程序已经在运行，请先安全退出或强制停止。")
            return False

        script_path = script_path.expanduser().resolve()
        if not script_path.exists():
            self.parent.append_log(f"[ERR] 找不到后端脚本: {script_path}")
            return False

        mode_arg = MODE_TO_ARG.get(initial_cmd)
        if mode_arg is None:
            self.parent.append_log(f"[ERR] 不支持的初始命令: {initial_cmd}")
            return False

        process = QProcess(self.parent)
        process.setProgram(sys.executable)
        process.setArguments(["-u", str(script_path), "--mode", mode_arg] + extra_args)
        process.setWorkingDirectory(str(script_path.parent))

        env = QProcessEnvironment.systemEnvironment()
        env.insert("PYTHONUNBUFFERED", "1")
        # 保证后端脚本优先从自身目录导入 GUIyemian_7motors_linux.py、USBCANFD_gai.py 等依赖。
        old_pythonpath = env.value("PYTHONPATH")
        new_pythonpath = str(script_path.parent)
        if old_pythonpath:
            new_pythonpath = new_pythonpath + os.pathsep + old_pythonpath
        env.insert("PYTHONPATH", new_pythonpath)
        process.setProcessEnvironment(env)

        process.readyReadStandardOutput.connect(self._on_stdout)
        process.readyReadStandardError.connect(self._on_stderr)
        process.errorOccurred.connect(self._on_error)
        process.finished.connect(self._on_finished)

        self.process = process
        self.current_script = script_path

        cmd_text = " ".join([sys.executable, "-u", str(script_path), "--mode", mode_arg] + extra_args)
        self.parent.append_log(f"[GUI] 启动后端: {cmd_text}")
        process.start()

        if not process.waitForStarted(3000):
            self.parent.append_log("[ERR] 后端程序启动失败。")
            self.process = None
            return False

        self.parent.set_backend_running(True)
        self.parent.set_current_mode(initial_cmd)
        return True

    def send_command(self, cmd: str) -> None:
        if not self.is_running():
            self.parent.append_log("[WARN] 后端程序未运行，无法发送命令。")
            return

        if cmd not in (CMD_EXIT, CMD_TELEOP, CMD_RECORD, CMD_REPLAY):
            self.parent.append_log(f"[WARN] 无效命令: {cmd}")
            return

        assert self.process is not None
        self.process.write((cmd + "\n").encode("utf-8"))
        self.process.waitForBytesWritten(1000)

        if cmd == CMD_EXIT:
            self.parent.append_log("[GUI] 已发送安全退出命令 0。")
        else:
            self.parent.append_log(f"[GUI] 已发送切换命令 {cmd}: {MODE_TO_NAME[cmd]}")
            self.parent.set_current_mode(cmd)

    def terminate(self) -> None:
        if not self.is_running():
            return
        assert self.process is not None
        self.parent.append_log("[WARN] 正在强制终止后端程序。优先建议使用“安全退出”。")
        self.process.terminate()
        if not self.process.waitForFinished(2000):
            self.process.kill()

    def _append_process_text(self, prefix: str, data: bytes) -> None:
        text = bytes(data).decode("utf-8", errors="replace")
        if not text:
            return
        for line in text.splitlines():
            self.parent.append_log(f"{prefix}{line}" if prefix else line)

    def _on_stdout(self) -> None:
        if self.process is None:
            return
        self._append_process_text("", self.process.readAllStandardOutput().data())

    def _on_stderr(self) -> None:
        if self.process is None:
            return
        self._append_process_text("[STDERR] ", self.process.readAllStandardError().data())

    def _on_error(self, error) -> None:  # noqa: ANN001 - Qt enum across versions
        self.parent.append_log(f"[ERR] QProcess 错误: {error}")

    def _on_finished(self, exit_code: int, exit_status) -> None:  # noqa: ANN001 - Qt enum across versions
        self.parent.append_log(f"[GUI] 后端程序已退出，exit_code={exit_code}, exit_status={exit_status}")
        self.parent.set_backend_running(False)
        self.parent.set_current_mode(None)
        self.process = None


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("弱双向遥操作 / 记录 / 示教回放控制界面")
        self.resize(1180, 780)

        self.proc = TeleoperationProcessController(self)
        self.current_mode: Optional[str] = None

        self._build_ui()
        self._update_button_state()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._update_button_state)
        self.ui_timer.start(500)

    # ------------------------------------------------------------------
    # UI 构建
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)

        splitter = QSplitter(Qt.Vertical)
        top = QWidget()
        top_layout = QHBoxLayout(top)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # 日志框需要在 _build_options_group() 之前创建，
        # 因为“清空日志”按钮会连接 self.log_box.clear。
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(6000)

        self._build_script_group(left_layout)
        self._build_start_group(left_layout)
        self._build_switch_group(left_layout)
        self._build_options_group(left_layout)
        left_layout.addStretch(1)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self._build_status_group(right_layout)
        self._build_hint_group(right_layout)
        right_layout.addStretch(1)

        top_layout.addWidget(left_panel, 0)
        top_layout.addWidget(right_panel, 1)

        splitter.addWidget(top)
        splitter.addWidget(self.log_box)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        root.addWidget(splitter)
        self.setCentralWidget(central)

    def _build_script_group(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("后端脚本")
        layout = QGridLayout(group)

        default_script = Path(__file__).resolve().parent / "teleoperation_shuangxiangshijiao_linux.py"

        self.script_edit = QLineEdit(str(default_script))
        self.btn_browse_script = QPushButton("选择脚本")
        self.btn_browse_script.clicked.connect(self.choose_script)

        layout.addWidget(QLabel("脚本路径"), 0, 0)
        layout.addWidget(self.script_edit, 0, 1)
        layout.addWidget(self.btn_browse_script, 0, 2)

        parent_layout.addWidget(group)

    def _build_start_group(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("启动")
        layout = QGridLayout(group)

        self.btn_start_teleop = QPushButton("启动遥操作")
        self.btn_start_record = QPushButton("启动并记录")
        self.btn_start_replay = QPushButton("启动回放")

        self.btn_start_teleop.clicked.connect(lambda: self.start_backend(CMD_TELEOP))
        self.btn_start_record.clicked.connect(lambda: self.start_backend(CMD_RECORD))
        self.btn_start_replay.clicked.connect(lambda: self.start_backend(CMD_REPLAY))

        layout.addWidget(self.btn_start_teleop, 0, 0)
        layout.addWidget(self.btn_start_record, 0, 1)
        layout.addWidget(self.btn_start_replay, 0, 2)

        parent_layout.addWidget(group)

    def _build_switch_group(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("运行中切换")
        layout = QGridLayout(group)

        self.btn_switch_teleop = QPushButton("切到遥操作 1")
        self.btn_switch_record = QPushButton("切到记录 2")
        self.btn_switch_replay = QPushButton("切到回放 3")
        self.btn_safe_exit = QPushButton("安全退出 0")
        self.btn_force_stop = QPushButton("强制停止")

        self.btn_switch_teleop.clicked.connect(lambda: self.proc.send_command(CMD_TELEOP))
        self.btn_switch_record.clicked.connect(lambda: self.proc.send_command(CMD_RECORD))
        self.btn_switch_replay.clicked.connect(lambda: self.proc.send_command(CMD_REPLAY))
        self.btn_safe_exit.clicked.connect(lambda: self.proc.send_command(CMD_EXIT))
        self.btn_force_stop.clicked.connect(self.force_stop_backend)

        layout.addWidget(self.btn_switch_teleop, 0, 0)
        layout.addWidget(self.btn_switch_record, 0, 1)
        layout.addWidget(self.btn_switch_replay, 0, 2)
        layout.addWidget(self.btn_safe_exit, 1, 0, 1, 2)
        layout.addWidget(self.btn_force_stop, 1, 2)

        parent_layout.addWidget(group)

    def _build_options_group(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("记录 / 回放选项")
        layout = QGridLayout(group)

        self.record_file_edit = QLineEdit()
        self.record_file_edit.setPlaceholderText("可选：记录文件名，例如 test1.csv；为空则自动生成")
        self.btn_browse_record = QPushButton("选择保存名")
        self.btn_browse_record.clicked.connect(self.choose_record_file)

        self.replay_file_edit = QLineEdit()
        self.replay_file_edit.setPlaceholderText("可选：回放 CSV 文件；为空则后端自动找最新记录")
        self.btn_browse_replay = QPushButton("选择回放文件")
        self.btn_browse_replay.clicked.connect(self.choose_replay_file)

        self.replay_source_combo = QComboBox()
        self.replay_source_combo.addItems(["actual", "target"])
        self.replay_source_combo.setCurrentText("actual")

        self.replay_speed_spin = QDoubleSpinBox()
        self.replay_speed_spin.setRange(0.05, 10.0)
        self.replay_speed_spin.setDecimals(3)
        self.replay_speed_spin.setSingleStep(0.1)
        self.replay_speed_spin.setValue(1.0)

        self.auto_scroll_check = QCheckBox("日志自动滚动")
        self.auto_scroll_check.setChecked(True)
        self.btn_clear_log = QPushButton("清空日志")
        self.btn_clear_log.clicked.connect(self.log_box.clear)

        layout.addWidget(QLabel("记录文件"), 0, 0)
        layout.addWidget(self.record_file_edit, 0, 1)
        layout.addWidget(self.btn_browse_record, 0, 2)

        layout.addWidget(QLabel("回放文件"), 1, 0)
        layout.addWidget(self.replay_file_edit, 1, 1)
        layout.addWidget(self.btn_browse_replay, 1, 2)

        layout.addWidget(QLabel("回放源"), 2, 0)
        layout.addWidget(self.replay_source_combo, 2, 1)

        layout.addWidget(QLabel("回放速度倍率"), 3, 0)
        layout.addWidget(self.replay_speed_spin, 3, 1)

        layout.addWidget(self.auto_scroll_check, 4, 0, 1, 2)
        layout.addWidget(self.btn_clear_log, 4, 2)

        parent_layout.addWidget(group)

    def _build_status_group(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("状态")
        layout = QGridLayout(group)

        self.backend_state_label = QLabel("后端状态：未运行")
        self.mode_state_label = QLabel("当前模式：无")
        self.script_state_label = QLabel("调用方式：python3 -u teleoperation_shuangxiangshijiao_linux.py")

        layout.addWidget(self.backend_state_label, 0, 0)
        layout.addWidget(self.mode_state_label, 1, 0)
        layout.addWidget(self.script_state_label, 2, 0)

        parent_layout.addWidget(group)

    def _build_hint_group(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("使用说明")
        layout = QVBoxLayout(group)

        hint = QLabel(
            "1. 先确认 teleoperation_shuangxiangshijiao_linux.py 及其依赖文件在同一目录。\n"
            "2. 点击“启动遥操作 / 启动并记录 / 启动回放”启动后端。\n"
            "3. 后端运行后，可用“切到遥操作/记录/回放”按钮发送 1/2/3。\n"
            "4. 停止时优先点击“安全退出 0”，不要直接强制停止。\n"
            "5. 回放文件为空时，后端会按原逻辑在 trajectory 文件夹中寻找最新记录。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        parent_layout.addWidget(group)

    # ------------------------------------------------------------------
    # UI 动作
    # ------------------------------------------------------------------
    def append_log(self, msg: str) -> None:
        self.log_box.appendPlainText(str(msg))
        if self.auto_scroll_check.isChecked():
            cursor = self.log_box.textCursor()
            cursor.movePosition(QTextCursor.End)
            self.log_box.setTextCursor(cursor)

    def choose_script(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 teleoperation_shuangxiangshijiao_linux.py",
            str(Path(self.script_edit.text()).expanduser().parent if self.script_edit.text() else Path.cwd()),
            "Python Files (*.py);;All Files (*)",
        )
        if path:
            self.script_edit.setText(path)

    def choose_record_file(self) -> None:
        base_dir = Path(self.script_edit.text()).expanduser().resolve().parent if self.script_edit.text() else Path.cwd()
        default_dir = base_dir / "trajectory"
        default_dir.mkdir(parents=True, exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(
            self,
            "选择记录文件名",
            str(default_dir / "teach_record_gui.csv"),
            "CSV Files (*.csv);;All Files (*)",
        )
        if path:
            self.record_file_edit.setText(path)

    def choose_replay_file(self) -> None:
        base_dir = Path(self.script_edit.text()).expanduser().resolve().parent if self.script_edit.text() else Path.cwd()
        default_dir = base_dir / "trajectory"
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择回放轨迹 CSV",
            str(default_dir if default_dir.exists() else base_dir),
            "CSV Files (*.csv);;All Files (*)",
        )
        if path:
            self.replay_file_edit.setText(path)

    def build_extra_args(self, initial_cmd: str) -> list[str]:
        args: list[str] = []

        record_file = self.record_file_edit.text().strip()
        if record_file and initial_cmd in (CMD_RECORD, CMD_TELEOP):
            args.extend(["--record-file", record_file])

        replay_file = self.replay_file_edit.text().strip()
        if replay_file and initial_cmd == CMD_REPLAY:
            args.extend(["--replay-file", replay_file])

        if initial_cmd == CMD_REPLAY:
            args.extend(["--replay-source", self.replay_source_combo.currentText().strip()])
            args.extend(["--replay-speed", f"{self.replay_speed_spin.value():.6f}"])

        return args

    def start_backend(self, initial_cmd: str) -> None:
        script = Path(self.script_edit.text().strip() or "teleoperation_shuangxiangshijiao_linux.py")
        extra_args = self.build_extra_args(initial_cmd)
        self.proc.start(script, initial_cmd, extra_args)

    def force_stop_backend(self) -> None:
        if not self.proc.is_running():
            return
        reply = QMessageBox.warning(
            self,
            "确认强制停止",
            "强制停止不会执行后端完整清理流程。\n优先建议点击“安全退出 0”。\n\n是否仍然强制停止？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.proc.terminate()

    def set_backend_running(self, running: bool) -> None:
        self.backend_state_label.setText("后端状态：运行中" if running else "后端状态：未运行")
        self._update_button_state()

    def set_current_mode(self, cmd: Optional[str]) -> None:
        self.current_mode = cmd
        if cmd is None:
            self.mode_state_label.setText("当前模式：无")
        else:
            self.mode_state_label.setText(f"当前模式：{MODE_TO_NAME.get(cmd, cmd)}")

    def _update_button_state(self) -> None:
        running = self.proc.is_running()

        self.btn_start_teleop.setEnabled(not running)
        self.btn_start_record.setEnabled(not running)
        self.btn_start_replay.setEnabled(not running)
        self.btn_browse_script.setEnabled(not running)

        self.btn_switch_teleop.setEnabled(running)
        self.btn_switch_record.setEnabled(running)
        self.btn_switch_replay.setEnabled(running)
        self.btn_safe_exit.setEnabled(running)
        self.btn_force_stop.setEnabled(running)

        # 运行中不建议修改后端启动参数，避免误以为会即时生效。
        self.record_file_edit.setEnabled(not running)
        self.replay_file_edit.setEnabled(not running)
        self.btn_browse_record.setEnabled(not running)
        self.btn_browse_replay.setEnabled(not running)
        self.replay_source_combo.setEnabled(not running)
        self.replay_speed_spin.setEnabled(not running)

    def closeEvent(self, event) -> None:  # noqa: ANN001 - Qt event type
        if not self.proc.is_running():
            event.accept()
            return

        reply = QMessageBox.question(
            self,
            "退出确认",
            "后端遥操作程序仍在运行。\n\n点击 Yes 会先发送安全退出命令 0；若后端长时间不退出，可再次打开后强制停止。\n是否退出？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            event.ignore()
            return

        self.proc.send_command(CMD_EXIT)
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()