# -*- coding: utf-8 -*-
"""
Modbus RTU 485 地址扫描工具
用法: python modbus_scan_addr.py [COM口] [波特率]
默认: COM14  115200 8N1
用途: 扫描传感器在哪个 Modbus 地址上响应 (读取输入寄存器0x0000, 功能码04H)
"""
import sys, time, struct
import serial
import serial.tools.list_ports as lp

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM14"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
REG  = 0x0000          # 输入寄存器 0x0000 = 实时浓度
QTY  = 0x0001
FN   = 0x04            # 读输入寄存器
RESP_BYTES = 7         # 正常响应: addr+fn+count(1)+data(2)+crc(2)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def make_frame(addr, fn=FN, reg=REG, qty=QTY) -> bytes:
    pdu = bytes([addr, fn]) + struct.pack(">HH", reg, qty)
    return pdu + struct.pack("<H", crc16(pdu))


def check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    return crc16(frame[:-2]) == struct.unpack("<H", frame[-2:])[0]


def main():
    print(f"打开 {PORT} @ {BAUD} 8N1 ...")
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1,
                        timeout=0.06, write_timeout=0.2)
    ser.reset_input_buffer()
    print("串口已打开:", ser.name, "\n")

    # ---------- 阶段0: 先静听 1.5s, 看是否有主动推送帧 (TTL 订阅模式特征) ----------
    print("== 阶段0: 静听 1.5s 检查是否有主动上发的数据 ==")
    ser.reset_input_buffer()
    listen_start = time.time()
    got = b""
    while time.time() - listen_start < 1.5:
        got += ser.read(ser.in_waiting or 1)
    if got:
        print("  收到主动数据(可能是 TTL 订阅帧或其它):", got.hex(" "))
    else:
        print("  无主动数据 (正常, 485 为问答式)")
    ser.reset_input_buffer()
    print()

    # ---------- 阶段1: 扫描地址 1~247 ----------
    print("== 阶段1: 扫描 Modbus 地址 1~247 (功能码04H 读浓度) ==")
    found = []          # (addr, resp_hex)
    exceptions = []     # 异常响应也说明该地址存在
    echoes = []         # 收到完全回显(疑似自环/接线问题)
    for addr in range(1, 248):
        frame = make_frame(addr)
        ser.reset_input_buffer()
        ser.write(frame)
        # 在超时窗口内收满一帧
        resp = b""
        deadline = time.time() + 0.09
        while time.time() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                resp += chunk
                if len(resp) >= RESP_BYTES:      # 已足够一帧长度
                    break
        if not resp:
            continue
        if resp == frame:
            echoes.append((addr, resp.hex(" ")))
            continue
        if check_crc(resp) and resp[0] == addr:
            fn_r = resp[1]
            if fn_r == FN:
                found.append((addr, resp.hex(" ")))
                print(f"  [命中] 地址 {addr:3d}  响应: {resp.hex(' ')}")
            elif fn_r == (FN | 0x80):
                exceptions.append((addr, resp.hex(" ")))
                print(f"  [异常] 地址 {addr:3d}  异常码响应: {resp.hex(' ')} (设备在, 但寄存器不可读)")
            else:
                print(f"  [其他] 地址 {addr:3d}  功能码0x{fn_r:02X} 响应: {resp.hex(' ')}")
        else:
            print(f"  [噪音] 地址 {addr:3d}  收到无法校验的数据: {resp.hex(' ')}")

    # ---------- 阶段2: 快速复核命中的地址 (再读一次确认稳定) ----------
    print("\n== 阶段2: 复核 ==")
    for addr, _ in found:
        frame = make_frame(addr)
        ser.reset_input_buffer()
        ser.write(frame)
        time.sleep(0.05)
        resp = ser.read(20)
        ok = check_crc(resp) and resp[0] == addr and resp[1] == FN
        print(f"  地址 {addr:3d}: {'OK  ' if ok else 'FAIL'} {resp.hex(' ')}")

    ser.close()

    print("\n==================== 结论 ====================")
    if found:
        print(f"找到传感器, 响应地址: {[a for a,_ in found]}")
    elif exceptions:
        print(f"仅收到异常响应(地址存在但功能码/寄存器不可用): {[a for a,_ in exceptions]}")
    elif echoes:
        print(f"全部是回显(环回): 可能是 A/B 未接对或处于 TTL 自环。地址: {[a for a,_ in echoes]}")
    else:
        print("247 个地址均无响应!")
        print("可能原因:")
        print("  1. A/B 接反 (交换 485A/485B 再试)")
        print("  2. 实际是 TTL 电平而非 485 (CH340 是 TTL 适配器, 需接 485 转接板)")
        print("  3. 波特率不是 115200 (可尝试 9600/19200/57600)")
        print("  4. 设备曾被切到 UDP 协议模式 (规格书 05H 线圈3)")
        print("  5. 传感器硬件故障")


if __name__ == "__main__":
    main()
