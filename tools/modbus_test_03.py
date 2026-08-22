# -*- coding: utf-8 -*-
"""
Modbus RTU 通信测试
用法: python modbus_test_03.py [COM口] [波特率]
默认: COM14  115200 8N1

测试内容:
  1. 静听是否有主动数据
  2. 目标地址(默认 03) 功能码 04H 读输入寄存器 0x0000(实时浓度)   —— 规格书 2.3.3
  3. 目标地址(默认 03) 功能码 03H 读保持寄存器 0x0000(报警值)     —— 规格书 2.3.3
  4. 快速扫描地址 1~15, 确认设备实际响应的地址
"""
import sys, time, struct
import serial
import serial.tools.list_ports as lp

PORT   = sys.argv[1] if len(sys.argv) > 1 else "COM14"
BAUD   = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
TARGET = 0x03            # 被测设备地址


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


def make_frame(addr, fn, reg, qty) -> bytes:
    pdu = bytes([addr, fn]) + struct.pack(">HH", reg, qty)
    return pdu + struct.pack("<H", crc16(pdu))


def check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    return crc16(frame[:-2]) == struct.unpack("<H", frame[-2:])[0]


def tx_rx(ser, frame, wait_s=0.12, max_len=32):
    """发送一帧并等待收满响应"""
    ser.reset_input_buffer()
    ser.write(frame)
    resp = b""
    deadline = time.time() + wait_s
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            resp += chunk
            if len(resp) >= max_len:
                break
    return resp


def test_func(ser, addr, fn, label):
    """对指定地址执行一次功能码测试, 返回 (ok, hex, extra)"""
    print(f"  [{label}] -> 发送: {make_frame(addr, fn, 0x0000, 0x0001).hex(' ')}")
    resp = tx_rx(ser, make_frame(addr, fn, 0x0000, 0x0001))
    if not resp:
        print(f"    无响应 (timeout)")
        return False, "", ""
    if resp == make_frame(addr, fn, 0x0000, 0x0001):
        print(f"    收到完全回显: {resp.hex(' ')}  (疑似自环/接线问题)")
        return False, resp.hex(" "), "echo"
    if not check_crc(resp):
        print(f"    CRC 校验失败: {resp.hex(' ')}")
        return False, resp.hex(" "), "badcrc"
    if resp[0] != addr:
        print(f"    响应地址不匹配({resp[0]:#04x} != {addr:#04x}): {resp.hex(' ')}")
        return False, resp.hex(" "), "addr"
    rfn = resp[1]
    if rfn == (fn | 0x80):
        print(f"    设备在, 但返回异常码 0x{rfn:02X}(异常码 {resp[2]}): {resp.hex(' ')}")
        return False, resp.hex(" "), "exception"
    if rfn == fn:
        # 解析数据
        if len(resp) >= 5 and resp[2] == 0x02:
            val = struct.unpack(">H", resp[3:5])[0]
            print(f"    OK! 数据=0x{val:04X} ({val})  {resp.hex(' ')}")
        else:
            print(f"    功能码正确但长度异常: {resp.hex(' ')}")
        return True, resp.hex(" "), "ok"
    print(f"    功能码不匹配(收到 0x{rfn:02X}): {resp.hex(' ')}")
    return False, resp.hex(" "), "fn"


def main():
    print(f"打开 {PORT} @ {BAUD} 8N1 ...")
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1,
                        timeout=0.06, write_timeout=0.2)
    ser.reset_input_buffer()
    print("串口已打开:", ser.name, "\n")

    # ---------- 阶段0: 静听 ----------
    print("== 阶段0: 静听 1.5s 检查是否有主动上发数据 ==")
    ser.reset_input_buffer()
    listen_start = time.time()
    got = b""
    while time.time() - listen_start < 1.5:
        got += ser.read(ser.in_waiting or 1)
    if got:
        print("  收到主动数据:", got.hex(" "))
    else:
        print("  无主动数据 (正常, 485 为问答式)")
    ser.reset_input_buffer()
    print()

    # ---------- 阶段1: 目标地址 03 ----------
    print(f"== 阶段1: 目标地址 {TARGET} 功能码测试 ==")
    r1 = test_func(ser, TARGET, 0x04, "地址03 功能码04H 读浓度")
    time.sleep(0.05)
    r2 = test_func(ser, TARGET, 0x03, "地址03 功能码03H 读报警值")
    print()

    # ---------- 阶段2: 快速扫描地址 1~15 ----------
    print("== 阶段2: 扫描地址 1~15 (功能码04H 读浓度) ==")
    found = []
    for addr in range(1, 16):
        resp = tx_rx(ser, make_frame(addr, 0x04, 0x0000, 0x0001))
        if not resp:
            continue
        if resp == make_frame(addr, 0x04, 0x0000, 0x0001):
            print(f"  地址 {addr:3d}: 完全回显 {resp.hex(' ')} (疑似自环)")
            continue
        if check_crc(resp) and resp[0] == addr:
            rfn = resp[1]
            if rfn == 0x04:
                found.append(addr)
                print(f"  地址 {addr:3d}: 命中! {resp.hex(' ')}")
            elif rfn == 0x84:
                print(f"  地址 {addr:3d}: 异常响应 {resp.hex(' ')} (设备在)")
            else:
                print(f"  地址 {addr:3d}: 功能码0x{rfn:02X} {resp.hex(' ')}")
        else:
            print(f"  地址 {addr:3d}: 噪音/坏CRC {resp.hex(' ')}")
    print()

    ser.close()

    # ---------- 结论 ----------
    print("==================== 结论 ====================")
    ok1 = r1[0] and r1[3] == "ok"
    ok2 = r2[0] and r2[3] == "ok"
    if ok1 or ok2:
        if ok1:
            print(f"地址 {TARGET} Modbus 通信正常: 功能码04H 读浓度成功")
        if ok2:
            print(f"地址 {TARGET} Modbus 通信正常: 功能码03H 读报警值成功")
        if ok1 and ok2:
            print(f"→ 地址 {TARGET} 上 04H/03H 均响应, 通信完全正常")
    elif found:
        print(f"地址 {TARGET} 无响应, 但设备实际在地址: {found} (注意核对设备地址)")
    else:
        print(f"地址 {TARGET} 无响应, 且 1~15 均未找到设备")
        print("可能原因:")
        print("  1. A/B 接反 (交换 485A/485B 再试)")
        print("  2. 实际是 TTL 电平而非 485 (CH340 是 TTL 适配器, 需接 485 转接板)")
        print("  3. 波特率不是 115200 (可尝试 9600/19200/57600)")
        print("  4. 设备曾被切到 UDP 协议模式 (规格书 05H 线圈3, 拨回 Modbus)")
        print("  5. 设备地址不在 1~15 内")
        print("  6. 传感器硬件故障")


if __name__ == "__main__":
    main()
