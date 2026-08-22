import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("missing dependency: pyserial\ninstall with: pip install pyserial") from exc


FRAME_HEADER_H = 0x5A
FRAME_HEADER_L = 0xA5
CMD_WRITE_VP = 0x82
CMD_READ_VP = 0x83
PAGE_SWITCH_VP = 0x0084

PARITY_OPTIONS = {
    "N": serial.PARITY_NONE,
    "E": serial.PARITY_EVEN,
    "O": serial.PARITY_ODD,
}

BYTESIZE_OPTIONS = {
    "8": serial.EIGHTBITS,
    "7": serial.SEVENBITS,
}

STOPBITS_OPTIONS = {
    "1": serial.STOPBITS_ONE,
    "2": serial.STOPBITS_TWO,
}


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_u16(text: str) -> int:
    value = int(text.strip(), 0)
    if not 0 <= value <= 0xFFFF:
        raise ValueError("value out of range: 0..0xFFFF")
    return value


def parse_hex_stream(text: str) -> bytes:
    tokens = text.replace(",", " ").split()
    if not tokens:
        return b""
    return bytes(int(token, 16) & 0xFF for token in tokens)


def frame_from_command(cmd: int, payload: bytes) -> bytes:
    if len(payload) > 0xFF - 1:
        raise ValueError("payload too long")
    return bytes([FRAME_HEADER_H, FRAME_HEADER_L, 1 + len(payload), cmd]) + payload


def build_write_vp_frame(vp: int, words: list[int]) -> bytes:
    payload = bytearray()
    payload.extend(vp.to_bytes(2, "big"))
    for word in words:
        payload.extend(word.to_bytes(2, "big"))
    return frame_from_command(CMD_WRITE_VP, bytes(payload))


def build_read_vp_frame(vp: int, word_count: int) -> bytes:
    if not 0 < word_count <= 0xFF:
        raise ValueError("word count out of range: 1..255")
    payload = vp.to_bytes(2, "big") + bytes([word_count])
    return frame_from_command(CMD_READ_VP, payload)


def build_set_page_frame(page_id: int) -> bytes:
    return build_write_vp_frame(PAGE_SWITCH_VP, [0x5A01, page_id])


class DgusiiSerialClient:
    def __init__(self):
        self._serial = None
        self._rx_thread = None
        self._rx_stop = threading.Event()
        self._rx_queue = queue.Queue()
        self._rx_buf = bytearray()

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self, port: str, baudrate: int, bytesize: str, parity: str, stopbits: str, timeout: float) -> None:
        self.close()
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=BYTESIZE_OPTIONS[bytesize],
            parity=PARITY_OPTIONS[parity],
            stopbits=STOPBITS_OPTIONS[stopbits],
            timeout=timeout,
        )
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._rx_stop.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=0.3)
            self._rx_thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        self._rx_buf.clear()

    def write(self, frame: bytes) -> None:
        if not self.is_open():
            raise RuntimeError("serial port is not open")
        self._serial.write(frame)
        self._serial.flush()

    def read_messages(self) -> list[tuple[str, str]]:
        items = []
        while True:
            try:
                items.append(self._rx_queue.get_nowait())
            except queue.Empty:
                return items

    def _queue_raw(self, prefix: str, data: bytes) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._rx_queue.put(("raw", f"[{stamp}] {prefix} {hex_bytes(data)}"))

    def _queue_info(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._rx_queue.put(("info", f"[{stamp}] {text}"))

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            if self._serial is None:
                break
            try:
                chunk = self._serial.read(64)
            except serial.SerialException as exc:
                self._queue_info(f"serial error: {exc}")
                break

            if not chunk:
                continue

            self._queue_raw("RX:", chunk)
            self._rx_buf.extend(chunk)
            self._parse_frames()

    def _parse_frames(self) -> None:
        while len(self._rx_buf) >= 4:
            if self._rx_buf[0] != FRAME_HEADER_H or self._rx_buf[1] != FRAME_HEADER_L:
                del self._rx_buf[0]
                continue

            frame_len = 3 + self._rx_buf[2]
            if frame_len < 4:
                del self._rx_buf[0]
                continue
            if len(self._rx_buf) < frame_len:
                break

            frame = bytes(self._rx_buf[:frame_len])
            del self._rx_buf[:frame_len]
            self._queue_info(self._describe_frame(frame))

    def _describe_frame(self, frame: bytes) -> str:
        cmd = frame[3]
        payload = frame[4:]
        if cmd == CMD_READ_VP and len(payload) >= 3:
            vp = int.from_bytes(payload[0:2], "big")
            values = [int.from_bytes(payload[i:i + 2], "big") for i in range(2, len(payload), 2) if i + 1 < len(payload)]
            return f"frame cmd=0x{cmd:02X} read-vp addr=0x{vp:04X} values={values}"
        if cmd == CMD_WRITE_VP and len(payload) >= 2:
            vp = int.from_bytes(payload[0:2], "big")
            values = [int.from_bytes(payload[i:i + 2], "big") for i in range(2, len(payload), 2) if i + 1 < len(payload)]
            return f"frame cmd=0x{cmd:02X} write-vp addr=0x{vp:04X} values={values}"
        return f"frame cmd=0x{cmd:02X} payload={hex_bytes(payload)}"


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("DGUSII UART Host")
        self.client = DgusiiSerialClient()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.bytesize_var = tk.StringVar(value="8")
        self.parity_var = tk.StringVar(value="N")
        self.stopbits_var = tk.StringVar(value="1")
        self.timeout_var = tk.StringVar(value="0.1")

        self.write_vp_var = tk.StringVar(value="0x2000")
        self.write_words_var = tk.StringVar(value="0x1234")
        self.read_vp_var = tk.StringVar(value="0x2000")
        self.read_count_var = tk.StringVar(value="1")
        self.page_var = tk.StringVar(value="0")
        self.raw_cmd_var = tk.StringVar(value="0x82")
        self.raw_payload_var = tk.StringVar(value="20 00 12 34")

        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.poll_messages)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.BOTH, expand=True)

        serial_frame = ttk.LabelFrame(top, text="Serial")
        serial_frame.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(serial_frame, text="Port").grid(row=0, column=0, padx=4, pady=4, sticky=tk.W)
        self.port_box = ttk.Combobox(serial_frame, textvariable=self.port_var, width=16, state="readonly")
        self.port_box.grid(row=0, column=1, padx=4, pady=4, sticky=tk.W)
        ttk.Button(serial_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4, pady=4)

        ttk.Label(serial_frame, text="Baud").grid(row=0, column=3, padx=4, pady=4, sticky=tk.W)
        ttk.Combobox(serial_frame, textvariable=self.baud_var, width=10,
                     values=("9600", "19200", "38400", "57600", "115200")).grid(row=0, column=4, padx=4, pady=4)

        ttk.Label(serial_frame, text="Data").grid(row=1, column=0, padx=4, pady=4, sticky=tk.W)
        ttk.Combobox(serial_frame, textvariable=self.bytesize_var, width=6, state="readonly",
                     values=("8", "7")).grid(row=1, column=1, padx=4, pady=4, sticky=tk.W)
        ttk.Label(serial_frame, text="Parity").grid(row=1, column=2, padx=4, pady=4, sticky=tk.W)
        ttk.Combobox(serial_frame, textvariable=self.parity_var, width=6, state="readonly",
                     values=("N", "E", "O")).grid(row=1, column=3, padx=4, pady=4, sticky=tk.W)
        ttk.Label(serial_frame, text="Stop").grid(row=1, column=4, padx=4, pady=4, sticky=tk.W)
        ttk.Combobox(serial_frame, textvariable=self.stopbits_var, width=6, state="readonly",
                     values=("1", "2")).grid(row=1, column=5, padx=4, pady=4, sticky=tk.W)
        ttk.Label(serial_frame, text="Timeout").grid(row=1, column=6, padx=4, pady=4, sticky=tk.W)
        ttk.Entry(serial_frame, textvariable=self.timeout_var, width=8).grid(row=1, column=7, padx=4, pady=4, sticky=tk.W)

        ttk.Button(serial_frame, text="Open", command=self.open_port).grid(row=0, column=5, padx=4, pady=4)
        ttk.Button(serial_frame, text="Close", command=self.close_port).grid(row=0, column=6, padx=4, pady=4)

        actions = ttk.Frame(top)
        actions.pack(fill=tk.X, pady=(0, 8))

        write_frame = ttk.LabelFrame(actions, text="Write VP")
        write_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(write_frame, text="VP").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(write_frame, textvariable=self.write_vp_var, width=12).grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(write_frame, text="Words").grid(row=0, column=2, padx=4, pady=4)
        ttk.Entry(write_frame, textvariable=self.write_words_var, width=32).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(write_frame, text="Send", command=self.send_write_vp).grid(row=0, column=4, padx=4, pady=4)

        read_frame = ttk.LabelFrame(actions, text="Read VP")
        read_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(read_frame, text="VP").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(read_frame, textvariable=self.read_vp_var, width=12).grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(read_frame, text="Count").grid(row=0, column=2, padx=4, pady=4)
        ttk.Entry(read_frame, textvariable=self.read_count_var, width=8).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(read_frame, text="Send", command=self.send_read_vp).grid(row=0, column=4, padx=4, pady=4)

        page_frame = ttk.LabelFrame(actions, text="Page")
        page_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(page_frame, text="Page ID").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(page_frame, textvariable=self.page_var, width=12).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(page_frame, text="Set Page", command=self.send_set_page).grid(row=0, column=2, padx=4, pady=4)

        raw_frame = ttk.LabelFrame(actions, text="Raw")
        raw_frame.pack(fill=tk.X)
        ttk.Label(raw_frame, text="CMD").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(raw_frame, textvariable=self.raw_cmd_var, width=10).grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(raw_frame, text="Payload Hex").grid(row=0, column=2, padx=4, pady=4)
        ttk.Entry(raw_frame, textvariable=self.raw_payload_var, width=36).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(raw_frame, text="Send Raw", command=self.send_raw).grid(row=0, column=4, padx=4, pady=4)

        log_frame = ttk.LabelFrame(top, text="Log")
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=20, wrap="none")
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def refresh_ports(self) -> None:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def open_port(self) -> None:
        try:
            self.client.open(
                port=self.port_var.get(),
                baudrate=int(self.baud_var.get(), 0),
                bytesize=self.bytesize_var.get(),
                parity=self.parity_var.get(),
                stopbits=self.stopbits_var.get(),
                timeout=float(self.timeout_var.get()),
            )
        except Exception as exc:
            messagebox.showerror("Open Port", str(exc))
            return
        self.append_log(f"opened {self.port_var.get()} @ {self.baud_var.get()} {self.bytesize_var.get()}{self.parity_var.get()}{self.stopbits_var.get()}")

    def close_port(self) -> None:
        self.client.close()
        self.append_log("port closed")

    def send_write_vp(self) -> None:
        try:
            vp = parse_u16(self.write_vp_var.get())
            words = [parse_u16(token) for token in self.write_words_var.get().replace(",", " ").split()]
            if not words:
                raise ValueError("at least one word is required")
            self.send_frame(build_write_vp_frame(vp, words))
        except Exception as exc:
            messagebox.showerror("Write VP", str(exc))

    def send_read_vp(self) -> None:
        try:
            vp = parse_u16(self.read_vp_var.get())
            count = int(self.read_count_var.get(), 0)
            self.send_frame(build_read_vp_frame(vp, count))
        except Exception as exc:
            messagebox.showerror("Read VP", str(exc))

    def send_set_page(self) -> None:
        try:
            page_id = parse_u16(self.page_var.get())
            self.send_frame(build_set_page_frame(page_id))
        except Exception as exc:
            messagebox.showerror("Set Page", str(exc))

    def send_raw(self) -> None:
        try:
            cmd = int(self.raw_cmd_var.get(), 0) & 0xFF
            payload = parse_hex_stream(self.raw_payload_var.get())
            self.send_frame(frame_from_command(cmd, payload))
        except Exception as exc:
            messagebox.showerror("Send Raw", str(exc))

    def send_frame(self, frame: bytes) -> None:
        try:
            self.client.write(frame)
        except Exception as exc:
            messagebox.showerror("Send Frame", str(exc))
            return
        self.append_log(f"TX: {hex_bytes(frame)}")

    def append_log(self, text: str) -> None:
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)

    def poll_messages(self) -> None:
        for kind, text in self.client.read_messages():
            self.append_log(text)
        self.root.after(100, self.poll_messages)

    def on_close(self) -> None:
        self.client.close()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
