# ==========================================
# 文件名: camera_control_gui.py
# 描  述: 四路偏振相机控制客户端 (极速稳定版)
# ==========================================

import socket
import struct
import threading
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, scrolledtext, ttk


SERVER_IP = "192.168.1.123"
SERVER_PORT = 2000
TIMEOUT = 2

CMD_TRANSFORM_ONLY = 0x00
CMD_RAW = 0x01
CMD_AOLP = 0x02
CMD_DOLP = 0x03
CMD_AUTO_EXP = 0x04
CMD_MANUAL_EXP = 0x05
CMD_S0 = 0x10
CMD_I0 = 0x11
CMD_I45 = 0x12
CMD_I90 = 0x13
CMD_I135 = 0x14
CMD_SET_AFFINE = 0x30

TRIGGER_MODE_SOFT_CONTINUOUS = 0
TRIGGER_MODE_HARD_LINE = 1
TRIGGER_MODE_NO_CHANGE = 255

CAM_NAMES = ["左下 BL", "左上 TL", "右下 BR", "右上 TR"]
DEFAULT_AFFINE = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_command(cam_id, command, gain=0, exposure_us=0,
                  trans_x=0, trans_y=0, rotation=0,
                  trigger_mode=TRIGGER_MODE_NO_CHANGE,
                  bitrate_mbps=0, expected_gray_value=120,
                  affine_params=None):
    if affine_params is None:
        affine_params = DEFAULT_AFFINE

    packet_without_crc = struct.pack(
        "<H H B B B I h h H B H B 6f",
        0x55AA,
        43,
        cam_id,
        command,
        gain,
        exposure_us,
        trans_x,
        trans_y,
        rotation,
        trigger_mode,
        bitrate_mbps,
        expected_gray_value,
        *affine_params,
    )
    crc = crc16(packet_without_crc)
    return packet_without_crc + struct.pack("<H", crc)


class CameraControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("四路偏振相机控制客户端")
        self.root.geometry("1120x860")
        self.root.resizable(True, True)

        self.server_ip = tk.StringVar(value=SERVER_IP)
        self.server_port = tk.IntVar(value=SERVER_PORT)
        self.connected = False
        self.selected_cam = 0

        self.cam_states = [{"affine_params": list(DEFAULT_AFFINE)} for _ in range(4)]
        self.expected_gray_var = tk.StringVar(value="120")
        self.affine_vars = [tk.StringVar(value=f"{v:.6f}") for v in DEFAULT_AFFINE]

        self.setup_ui()

    def setup_ui(self):
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_container = ttk.Frame(main_paned)
        main_paned.add(left_container, weight=1)

        self.left_canvas = tk.Canvas(left_container, highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(left_container, orient=tk.VERTICAL, command=self.left_canvas.yview)
        self.left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(self.left_canvas)
        self.left_window = self.left_canvas.create_window((0, 0), window=left_frame, anchor="nw")

        def _update_left_scrollregion(_event=None):
            self.left_canvas.configure(scrollregion=self.left_canvas.bbox("all"))

        def _resize_left_frame(event):
            self.left_canvas.itemconfigure(self.left_window, width=event.width)

        left_frame.bind("<Configure>", _update_left_scrollregion)
        self.left_canvas.bind("<Configure>", _resize_left_frame)

        config_frame = ttk.LabelFrame(left_frame, text="服务器配置", padding=6)
        config_frame.pack(fill=tk.X, padx=2, pady=2)

        cfg_row = ttk.Frame(config_frame)
        cfg_row.pack(fill=tk.X)
        ttk.Label(cfg_row, text="IP:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(cfg_row, textvariable=self.server_ip, width=14).pack(side=tk.LEFT, padx=2)
        ttk.Label(cfg_row, text="端口:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(cfg_row, textvariable=self.server_port, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Button(cfg_row, text="测试连接", command=self.test_connection).pack(side=tk.LEFT, padx=8)
        self.status_label = ttk.Label(cfg_row, text="● 未连接", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=8)

        top_ctrl_frame = ttk.Frame(left_frame)
        top_ctrl_frame.pack(fill=tk.X, padx=2, pady=2)

        trigger_frame = ttk.LabelFrame(top_ctrl_frame, text="触发模式（全局）", padding=6)
        trigger_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 2))
        ttk.Button(trigger_frame, text="自由采集", command=self.apply_trigger_soft).pack(fill=tk.X, pady=2)
        ttk.Button(trigger_frame, text="硬触发", command=self.apply_trigger_hard).pack(fill=tk.X, pady=2)

        bitrate_frame = ttk.LabelFrame(top_ctrl_frame, text="码率（全局）", padding=6)
        bitrate_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=(2, 0))
        bitrate_row = ttk.Frame(bitrate_frame)
        bitrate_row.pack(fill=tk.X, pady=2)
        self.bitrate_var = tk.StringVar(value="4")
        ttk.Entry(bitrate_row, textvariable=self.bitrate_var, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Label(bitrate_row, text="Mbps").pack(side=tk.LEFT, padx=2)
        ttk.Button(bitrate_frame, text="应用", command=self.apply_bitrate).pack(fill=tk.X, pady=2)

        mid_ctrl_frame = ttk.Frame(left_frame)
        mid_ctrl_frame.pack(fill=tk.X, padx=2, pady=2)

        mode_frame = ttk.LabelFrame(mid_ctrl_frame, text="图像模式（全局）", padding=6)
        mode_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 2))
        self.mode_var = tk.IntVar(value=1)
        ttk.Radiobutton(mode_frame, text="RAW", variable=self.mode_var, value=1).grid(row=0, column=0, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(mode_frame, text="AOLP", variable=self.mode_var, value=2).grid(row=0, column=1, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(mode_frame, text="DOLP", variable=self.mode_var, value=3).grid(row=1, column=0, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(mode_frame, text="S0", variable=self.mode_var, value=4).grid(row=1, column=1, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(mode_frame, text="I0", variable=self.mode_var, value=5).grid(row=2, column=0, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(mode_frame, text="I45", variable=self.mode_var, value=6).grid(row=2, column=1, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(mode_frame, text="I90", variable=self.mode_var, value=7).grid(row=3, column=0, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(mode_frame, text="I135", variable=self.mode_var, value=8).grid(row=3, column=1, sticky=tk.W, padx=4, pady=1)
        ttk.Button(mode_frame, text="应用", command=self.apply_mode).grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=4)

        exp_frame = ttk.LabelFrame(mid_ctrl_frame, text="曝光（全局）", padding=6)
        exp_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(2, 0))
        self.exp_mode_var = tk.IntVar(value=0)
        ttk.Radiobutton(exp_frame, text="自动", variable=self.exp_mode_var, value=0).grid(row=0, column=0, sticky=tk.W, padx=4, pady=1)
        ttk.Radiobutton(exp_frame, text="手动", variable=self.exp_mode_var, value=1).grid(row=0, column=1, sticky=tk.W, padx=4, pady=1)
        ttk.Label(exp_frame, text="曝光(us):").grid(row=1, column=0, sticky=tk.W, padx=2, pady=2)
        self.exposure_var = tk.StringVar(value="20000")
        ttk.Entry(exp_frame, textvariable=self.exposure_var, width=10).grid(row=1, column=1, padx=2, pady=2)
        ttk.Label(exp_frame, text="增益:").grid(row=2, column=0, sticky=tk.W, padx=2, pady=2)
        self.gain_var = tk.StringVar(value="0.0")
        ttk.Entry(exp_frame, textvariable=self.gain_var, width=10).grid(row=2, column=1, padx=2, pady=2)
        ttk.Label(exp_frame, text="目标灰度:").grid(row=3, column=0, sticky=tk.W, padx=2, pady=2)
        ttk.Entry(exp_frame, textvariable=self.expected_gray_var, width=10).grid(row=3, column=1, padx=2, pady=2)
        ttk.Button(exp_frame, text="应用", command=self.apply_exposure).grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=4)

        cam_frame = ttk.LabelFrame(left_frame, text="相机选择", padding=6)
        cam_frame.pack(fill=tk.X, padx=2, pady=2)
        self.cam_buttons = []
        btn_frame = ttk.Frame(cam_frame)
        btn_frame.pack(fill=tk.X)
        for i, name in enumerate(CAM_NAMES):
            btn = ttk.Button(btn_frame, text=name, width=9, command=lambda idx=i: self.select_camera(idx))
            btn.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
            self.cam_buttons.append(btn)
        self.selected_cam_label = ttk.Label(cam_frame, text="当前: 相机1 (左下 BL)", font=("Arial", 10, "bold"))
        self.selected_cam_label.pack(pady=4)

        transform_frame = ttk.LabelFrame(left_frame, text="平移 / 旋转（单相机）", padding=6)
        transform_frame.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(transform_frame, text="X:").grid(row=0, column=0, sticky=tk.W, padx=2, pady=2)
        self.trans_x_var = tk.StringVar(value="0")
        ttk.Entry(transform_frame, textvariable=self.trans_x_var, width=8).grid(row=0, column=1, padx=2, pady=2)
        ttk.Label(transform_frame, text="Y:").grid(row=0, column=2, sticky=tk.W, padx=6, pady=2)
        self.trans_y_var = tk.StringVar(value="0")
        ttk.Entry(transform_frame, textvariable=self.trans_y_var, width=8).grid(row=0, column=3, padx=2, pady=2)
        ttk.Label(transform_frame, text="角度:").grid(row=0, column=4, sticky=tk.W, padx=6, pady=2)
        self.rotation_var = tk.StringVar(value="0.0")
        ttk.Entry(transform_frame, textvariable=self.rotation_var, width=8).grid(row=0, column=5, padx=2, pady=2)
        ttk.Button(transform_frame, text="应用", command=self.apply_transform).grid(row=1, column=0, columnspan=6, sticky=tk.EW, pady=4)

        affine_frame = ttk.LabelFrame(left_frame, text="仿射矩阵 2x3（单相机）", padding=6)
        affine_frame.pack(fill=tk.X, padx=2, pady=2)
        labels = ["a", "b", "c", "d", "e", "f"]
        for i, label in enumerate(labels):
            row = i // 3
            col = (i % 3) * 2
            ttk.Label(affine_frame, text=f"{label}:").grid(row=row, column=col, sticky=tk.W, padx=2, pady=2)
            ttk.Entry(affine_frame, textvariable=self.affine_vars[i], width=12).grid(row=row, column=col + 1, padx=2, pady=2)
        ttk.Button(affine_frame, text="应用仿射矩阵", command=self.apply_affine).grid(row=2, column=0, columnspan=6, sticky=tk.EW, pady=4)

        current_affine_frame = ttk.LabelFrame(left_frame, text="当前仿射矩阵（只读）", padding=6)
        current_affine_frame.pack(fill=tk.X, padx=2, pady=2)
        self.current_affine_line1 = tk.StringVar(value="[1.000000, 0.000000, 0.000]")
        self.current_affine_line2 = tk.StringVar(value="[0.000000, 1.000000, 0.000]")
        ttk.Label(current_affine_frame, textvariable=self.current_affine_line1, font=("Consolas", 10)).pack(anchor="w")
        ttk.Label(current_affine_frame, textvariable=self.current_affine_line2, font=("Consolas", 10)).pack(anchor="w")

        right_frame = ttk.Frame(main_paned)
        main_paned.add(right_frame, weight=2)

        notebook = ttk.Notebook(right_frame)
        notebook.pack(fill=tk.BOTH, expand=True)

        response_tab = ttk.Frame(notebook)
        notebook.add(response_tab, text="最新回复")
        resp_toolbar = ttk.Frame(response_tab)
        resp_toolbar.pack(fill=tk.X, pady=2)
        ttk.Button(resp_toolbar, text="清空", command=self.clear_response).pack(side=tk.RIGHT, padx=5)
        self.response_text = tk.Text(response_tab, height=10, width=60, state=tk.DISABLED)
        response_scrollbar = ttk.Scrollbar(response_tab, orient=tk.VERTICAL, command=self.response_text.yview)
        self.response_text.configure(yscrollcommand=response_scrollbar.set)
        self.response_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        response_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.response_text.tag_configure("header", font=("Arial", 10, "bold"))
        self.response_text.tag_configure("value", font=("Arial", 10))
        self.response_text.tag_configure("success", foreground="green", font=("Arial", 11, "bold"))
        self.response_text.tag_configure("error", foreground="red", font=("Arial", 11, "bold"))

        log_tab = ttk.Frame(notebook)
        notebook.add(log_tab, text="通信日志")
        log_toolbar = ttk.Frame(log_tab)
        log_toolbar.pack(fill=tk.X, pady=2)
        ttk.Button(log_toolbar, text="清空日志", command=lambda: self.log_text.delete(1.0, tk.END)).pack(side=tk.RIGHT, padx=5)
        self.log_text = scrolledtext.ScrolledText(log_tab, height=10, width=60)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        self.select_camera(0)

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    def clear_response(self):
        self.response_text.config(state=tk.NORMAL)
        self.response_text.delete(1.0, tk.END)
        self.response_text.config(state=tk.DISABLED)

    def update_response_display(self, success, data_dict):
        self.response_text.config(state=tk.NORMAL)
        self.response_text.delete(1.0, tk.END)
        if success:
            self.response_text.insert(tk.END, "✓ 命令执行成功\n\n", "success")
            self.response_text.insert(tk.END, "回复详情:\n", "header")
            self.response_text.insert(tk.END, "-" * 52 + "\n")
            for key, value in data_dict.items():
                if str(key).startswith("_"):
                    continue
                self.response_text.insert(tk.END, f"{key}: ", "header")
                self.response_text.insert(tk.END, f"{value}\n", "value")
        else:
            self.response_text.insert(tk.END, "✗ 命令执行失败\n\n", "error")
            self.response_text.insert(tk.END, f"错误信息: {data_dict.get('error', '未知错误')}\n", "value")
        self.response_text.config(state=tk.DISABLED)

    def update_batch_response_display(self, title, results):
        self.response_text.config(state=tk.NORMAL)
        self.response_text.delete(1.0, tk.END)
        ok_count = sum(1 for _, success, _ in results if success)
        self.response_text.insert(tk.END, f"{title}\n", "header")
        self.response_text.insert(tk.END, f"成功 {ok_count}/{len(results)}\n\n", "success" if ok_count == len(results) else "header")
        for cam_id, success, info in results:
            self.response_text.insert(tk.END, f"相机{cam_id}: ", "header")
            if success:
                self.response_text.insert(tk.END, "成功", "success")
                self.response_text.insert(tk.END, f" | 模式={info.get('图像类型', '-')} | 触发={info.get('触发模式', '-')} | 码率={info.get('码率', '-')}\n", "value")
            else:
                self.response_text.insert(tk.END, f"失败 | {info}\n", "error")
        self.response_text.config(state=tk.DISABLED)

    def select_camera(self, idx):
        self.selected_cam = idx
        self.selected_cam_label.config(text=f"当前: 相机{idx + 1} ({CAM_NAMES[idx]})")
        self.log(f"切换到相机{idx + 1} ({CAM_NAMES[idx]})")
        self.load_affine_editor_from_state(idx)
        self.update_current_affine_display(idx)

    def store_camera_state_from_response(self, data_dict):
        cam_id = data_dict.get("_cam_id_int")
        if not isinstance(cam_id, int) or not (1 <= cam_id <= 4):
            return
        affine_params = data_dict.get("_affine_params_raw")
        if affine_params is not None:
            self.cam_states[cam_id - 1]["affine_params"] = list(affine_params)
        if cam_id - 1 == self.selected_cam:
            self.update_current_affine_display(self.selected_cam)
            self.load_affine_editor_from_state(self.selected_cam)

    def load_affine_editor_from_state(self, idx):
        affine_params = self.cam_states[idx].get("affine_params", list(DEFAULT_AFFINE))
        for i, value in enumerate(affine_params):
            self.affine_vars[i].set(f"{value:.6f}")

    def update_current_affine_display(self, idx):
        affine_params = self.cam_states[idx].get("affine_params", list(DEFAULT_AFFINE))
        self.current_affine_line1.set(f"[{affine_params[0]:.6f}, {affine_params[1]:.6f}, {affine_params[2]:.3f}]")
        self.current_affine_line2.set(f"[{affine_params[3]:.6f}, {affine_params[4]:.6f}, {affine_params[5]:.3f}]")

    def validate_int(self, value, min_val, max_val, field_name):
        try:
            val = int(value)
        except ValueError:
            return None, f"{field_name}必须是有效的整数"
        if min_val <= val <= max_val:
            return val, None
        return None, f"{field_name}必须在 {min_val} ~ {max_val} 范围内"

    def validate_float(self, value, min_val, max_val, field_name):
        try:
            val = float(value)
        except ValueError:
            return None, f"{field_name}必须是有效的数字"
        if min_val <= val <= max_val:
            return val, None
        return None, f"{field_name}必须在 {min_val} ~ {max_val} 范围内"

    def parse_float(self, value, field_name):
        try:
            return float(value), None
        except ValueError:
            return None, f"{field_name}必须是有效的数字"

    def test_connection(self):
        def do_test():
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(TIMEOUT)
                packet = build_command(1, CMD_RAW, 0, 0, 0, 0, 0, TRIGGER_MODE_NO_CHANGE, 0, 120, list(DEFAULT_AFFINE))
                sock.sendto(packet, (self.server_ip.get(), self.server_port.get()))
                data, _ = sock.recvfrom(1024)
                self.connected = True
                self.root.after(0, lambda: self.status_label.config(text="● 已连接", foreground="green"))
                self.log("✓ 连接成功")
                if len(data) >= 56:
                    resp_data = self.parse_response(data)
                    self.store_camera_state_from_response(resp_data)
                    self.root.after(0, lambda: self.update_response_display(True, resp_data))
            except Exception as e:
                self.connected = False
                self.root.after(0, lambda: self.status_label.config(text="● 连接失败", foreground="red"))
                self.log(f"✗ 连接失败: {e}")
                self.root.after(0, lambda: self.update_response_display(False, {'error': str(e)}))
            finally:
                if sock is not None:
                    sock.close()
        threading.Thread(target=do_test, daemon=True).start()

    def parse_response(self, data):
        if len(data) < 56:
            raise ValueError(f"回复数据长度不足: {len(data)}")
        header = struct.unpack("<H", data[0:2])[0]
        length = struct.unpack("<H", data[2:4])[0]
        timestamp = struct.unpack("<Q", data[4:12])[0]
        cam_id = data[12]
        image_type = data[13]
        exp_type = data[14]
        gain_raw = data[15]
        gain = gain_raw / 10.0
        exposure = struct.unpack("<I", data[16:20])[0]
        trans_x = struct.unpack("<h", data[20:22])[0]
        trans_y = struct.unpack("<h", data[22:24])[0]
        rotation = struct.unpack("<H", data[24:26])[0] / 10.0
        trigger_mode = data[26]
        bitrate_mbps = struct.unpack("<H", data[27:29])[0]
        expected_gray_value = data[29]
        affine_params = list(struct.unpack("<6f", data[30:54]))
        crc = struct.unpack("<H", data[54:56])[0]

        image_types = {
            0: "未知", 1: "RAW", 2: "AOLP", 3: "DOLP",
            0x10: "S0", 0x11: "I0", 0x12: "I45", 0x13: "I90", 0x14: "I135",
        }
        exp_types = {0: "手动", 1: "自动"}
        trigger_types = {0: "自由采集", 1: "硬触发", 255: "不修改"}
        affine_str = f"[{affine_params[0]:.6f}, {affine_params[1]:.6f}, {affine_params[2]:.3f}; {affine_params[3]:.6f}, {affine_params[4]:.6f}, {affine_params[5]:.3f}]"

        return {
            "包头": f"0x{header:04X}",
            "包长": length,
            "时间戳": f"{timestamp} ms",
            "相机ID": cam_id,
            "图像类型": image_types.get(image_type, f"未知({image_type})"),
            "曝光类型": exp_types.get(exp_type, f"未知({exp_type})"),
            "增益": f"{gain_raw} ({gain:.1f})",
            "曝光时间": f"{exposure} us",
            "X平移": trans_x,
            "Y平移": trans_y,
            "旋转": f"{rotation:.1f}°",
            "触发模式": trigger_types.get(trigger_mode, f"未知({trigger_mode})"),
            "码率": f"{bitrate_mbps} Mbps",
            "目标灰度": expected_gray_value,
            "仿射矩阵": affine_str,
            "CRC": f"0x{crc:04X}",
            "_cam_id_int": cam_id,
            "_affine_params_raw": affine_params,
        }

    # ================= 核心重试机制 =================
    def send_command(self, cam_id, command, gain=0, exposure_us=0, trans_x=0, trans_y=0, rotation=0.0,
                     trigger_mode=TRIGGER_MODE_NO_CHANGE, bitrate_mbps=0, expected_gray_value=120,
                     affine_params=None):
        MAX_RETRIES = 3  # 最大重试次数
        
        gain_int = int(round(gain * 10.0))
        rotation_int = int(round(rotation * 10.0))
        packet = build_command(cam_id, command, gain_int, exposure_us, trans_x, trans_y, rotation_int,
                               trigger_mode, bitrate_mbps, expected_gray_value, affine_params)

        cmd_names = {
            CMD_TRANSFORM_ONLY: "参数", CMD_RAW: "RAW", CMD_AOLP: "AOLP", CMD_DOLP: "DOLP",
            CMD_S0: "S0", CMD_I0: "I0", CMD_I45: "I45", CMD_I90: "I90", CMD_I135: "I135",
            CMD_AUTO_EXP: "自动曝光", CMD_MANUAL_EXP: "手动曝光", CMD_SET_AFFINE: "仿射矩阵",
        }
        cmd_name = cmd_names.get(command, f"0x{command:02X}")
        self.log(f"发送: 相机{cam_id} [{cmd_name}] 增益={gain:.1f} 曝光={exposure_us}us X={trans_x} Y={trans_y} R={rotation:.1f}° 触发={trigger_mode} 码率={bitrate_mbps}Mbps 目标灰度={expected_gray_value}")

        for attempt in range(MAX_RETRIES):
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                
                # 触发模式切换硬件操作较慢，固定 20 秒；其他模式使用递增超时
                timeout = 20 if trigger_mode != TRIGGER_MODE_NO_CHANGE else TIMEOUT + attempt
                
                if trigger_mode != TRIGGER_MODE_NO_CHANGE and attempt == 0:
                    self.log(f"相机{cam_id} 触发切换中...")
                    
                sock.settimeout(timeout)

                sock.sendto(packet, (self.server_ip.get(), self.server_port.get()))
                data, _ = sock.recvfrom(1024)
                
                if len(data) >= 56:
                    resp_data = self.parse_response(data)
                    self.store_camera_state_from_response(resp_data)
                    
                    success_msg = "收到回复"
                    if attempt > 0:
                        success_msg += f" (第{attempt+1}次尝试成功)"
                        
                    self.log(f"{success_msg}: 相机{resp_data['相机ID']} 模式={resp_data['图像类型']} 触发={resp_data['触发模式']} 码率={resp_data['码率']}")
                    return True, resp_data
                return False, "收到无效回复数据"
                
            except socket.timeout:
                if attempt < MAX_RETRIES - 1:
                    self.log(f"⚠ 等待回复超时，正在进行第 {attempt + 2} 次重试...")
                    continue  # 进入下一次循环
                return False, "连续多次尝试，等待回复超时"
                
            except Exception as e:
                return False, str(e)
                
            finally:
                if sock is not None:
                    sock.close()
    # ===============================================

    def send_command_single_and_display(self, cam_id, command, gain=0, exposure_us=0, trans_x=0, trans_y=0, rotation=0.0,
                                        trigger_mode=TRIGGER_MODE_NO_CHANGE, bitrate_mbps=0, expected_gray_value=120,
                                        affine_params=None):
        success, info = self.send_command(cam_id, command, gain, exposure_us, trans_x, trans_y, rotation,
                                          trigger_mode, bitrate_mbps, expected_gray_value, affine_params)
        if success:
            self.root.after(0, lambda: self.update_response_display(True, info))
        else:
            self.root.after(0, lambda: self.update_response_display(False, {"error": info}))
        return success

    # ================= 核心逻辑：全局指令只发一次 =================
    def send_command_all(self, title, command, gain=0, exposure_us=0, trans_x=0, trans_y=0, rotation=0.0,
                         trigger_mode=TRIGGER_MODE_NO_CHANGE, bitrate_mbps=0, expected_gray_value=120):
        # C端已经实现了一次接收全局配置自动应用四路，因此客户端只需发送一次给相机1即可
        global_cam_id = 1
        
        success, info = self.send_command(global_cam_id, command, gain, exposure_us, trans_x, trans_y, rotation,
                                          trigger_mode, bitrate_mbps, expected_gray_value, list(DEFAULT_AFFINE))
        
        results = []
        for cam_id in range(1, 5):
            if success:
                results.append((cam_id, True, info))
            else:
                results.append((cam_id, False, info))
                
        if not success:
            self.log(f"⚠ 全局配置下发失败: {info}")

        ok_count = sum(1 for _, success, _ in results if success)
        self.root.after(0, lambda: self.update_batch_response_display(title, results))
        return ok_count
    # ==============================================================

    def apply_mode(self):
        mode_map = {
            1: CMD_RAW,
            2: CMD_AOLP,
            3: CMD_DOLP,
            4: CMD_S0,
            5: CMD_I0,
            6: CMD_I45,
            7: CMD_I90,
            8: CMD_I135,
        }
        command = mode_map.get(self.mode_var.get(), CMD_RAW)

        def do_send():
            ok_count = self.send_command_all(title="图像模式", command=command)
            self.log(f"✓ 图像模式已应用，成功 {ok_count}/4")
        threading.Thread(target=do_send, daemon=True).start()

    def apply_exposure(self):
        expected_gray, gray_err = self.validate_int(self.expected_gray_var.get(), 0, 255, "目标灰度")
        if gray_err:
            messagebox.showerror("输入错误", gray_err)
            return
        if self.exp_mode_var.get() == 0:
            command = CMD_AUTO_EXP
            exposure = 0
            gain = 0.0
            exp_desc = f"自动曝光 目标灰度={expected_gray}"
        else:
            command = CMD_MANUAL_EXP
            exp_val, exp_err = self.validate_int(self.exposure_var.get(), 20, 1000000, "曝光时间")
            if exp_err:
                messagebox.showerror("输入错误", exp_err)
                return
            gain_val, gain_err = self.validate_float(self.gain_var.get(), 0.0, 24.0, "增益")
            if gain_err:
                messagebox.showerror("输入错误", gain_err)
                return
            exposure = exp_val
            gain = gain_val
            exp_desc = f"手动曝光 {exposure}us 增益={gain:.1f}"

        def do_send():
            ok_count = self.send_command_all(title="曝光设置", command=command, gain=gain,
                                             exposure_us=exposure, expected_gray_value=expected_gray)
            self.log(f"✓ 曝光已应用：{exp_desc}，成功 {ok_count}/4")
        threading.Thread(target=do_send, daemon=True).start()

    def apply_transform(self):
        tx_val, tx_err = self.validate_int(self.trans_x_var.get(), -1000, 1000, "X平移")
        if tx_err:
            messagebox.showerror("输入错误", tx_err)
            return
        ty_val, ty_err = self.validate_int(self.trans_y_var.get(), -1000, 1000, "Y平移")
        if ty_err:
            messagebox.showerror("输入错误", ty_err)
            return
        rot_val, rot_err = self.validate_float(self.rotation_var.get(), 0.0, 360.0, "旋转角度")
        if rot_err:
            messagebox.showerror("输入错误", rot_err)
            return
        cam_id = self.selected_cam + 1
        cam_name = CAM_NAMES[self.selected_cam]

        def do_send():
            success = self.send_command_single_and_display(cam_id, CMD_TRANSFORM_ONLY, 0, 0, tx_val, ty_val, rot_val,
                                                           TRIGGER_MODE_NO_CHANGE, 0, 120, list(DEFAULT_AFFINE))
            if success:
                self.log(f"✓ 相机{cam_id} ({cam_name}) 平移/旋转已应用")
                self.root.after(0, lambda: self.trans_x_var.set("0"))
                self.root.after(0, lambda: self.trans_y_var.set("0"))
                self.root.after(0, lambda: self.rotation_var.set("0.0"))
        threading.Thread(target=do_send, daemon=True).start()

    def apply_affine(self):
        affine_params = []
        labels = ["a", "b", "c", "d", "e", "f"]
        for i, label in enumerate(labels):
            value, err = self.parse_float(self.affine_vars[i].get(), f"仿射参数 {label}")
            if err:
                messagebox.showerror("输入错误", err)
                return
            affine_params.append(value)
        cam_id = self.selected_cam + 1
        cam_name = CAM_NAMES[self.selected_cam]

        def do_send():
            success = self.send_command_single_and_display(cam_id, CMD_SET_AFFINE, 0, 0, 0, 0, 0.0,
                                                           TRIGGER_MODE_NO_CHANGE, 0, 120, affine_params)
            if success:
                self.log(f"✓ 相机{cam_id} ({cam_name}) 仿射矩阵已保存 [{affine_params[0]:.6f}, {affine_params[1]:.6f}, {affine_params[2]:.3f}; {affine_params[3]:.6f}, {affine_params[4]:.6f}, {affine_params[5]:.3f}]")
        threading.Thread(target=do_send, daemon=True).start()

    def apply_trigger_soft(self):
        def do_send():
            ok_count = self.send_command_all(title="自由采集", command=CMD_TRANSFORM_ONLY,
                                             trigger_mode=TRIGGER_MODE_SOFT_CONTINUOUS)
            self.log(f"✓ 自由采集已应用，成功 {ok_count}/4")
        threading.Thread(target=do_send, daemon=True).start()

    def apply_trigger_hard(self):
        if not messagebox.askyesno("确认", "切换到硬触发后，如果没有外部触发信号，画面会停止刷新。\n确定继续吗？"):
            return
        def do_send():
            ok_count = self.send_command_all(title="硬触发", command=CMD_TRANSFORM_ONLY,
                                             trigger_mode=TRIGGER_MODE_HARD_LINE)
            self.log(f"✓ 硬触发已应用，成功 {ok_count}/4")
        threading.Thread(target=do_send, daemon=True).start()

    def apply_bitrate(self):
        bitrate_val, bitrate_err = self.validate_int(self.bitrate_var.get(), 1, 200, "码率")
        if bitrate_err:
            messagebox.showerror("输入错误", bitrate_err)
            return
        def do_send():
            ok_count = self.send_command_all(title=f"码率 {bitrate_val} Mbps", command=CMD_TRANSFORM_ONLY,
                                             bitrate_mbps=bitrate_val)
            self.log(f"✓ 码率已应用：{bitrate_val} Mbps，成功 {ok_count}/4")
        threading.Thread(target=do_send, daemon=True).start()


def main():
    root = tk.Tk()
    CameraControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()