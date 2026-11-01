#!/usr/bin/env python3
"""串口原始探测工具。

用途：
  1. 不启动 ROS 的情况下，向 MiniPC 发送导航帧；
  2. 同时打印 MiniPC 返回的原始字节；
  3. 用遥控器移动底盘时，检查 MiniPC 到底有没有反馈。

它不依赖 ROS，不会被 chassis_bridge 的参数/话题干扰。

示例：
  # 只监听 20 秒，用遥控器移动底盘
  ros2 run chassis chassis_serial_probe --port /dev/ttyACM1 --seconds 20 --no-write

  # 发送 0.1 m/s 前进 10 秒，并打印返回字节
  ros2 run chassis chassis_serial_probe \
      --port /dev/ttyACM1 --seconds 10 --forward 0.1

  # 发送角速度 0.15 rad/s 旋转 10 秒
  ros2 run chassis chassis_serial_probe \
      --port /dev/ttyACM1 --seconds 10 --angular 0.15 --field w

注意：运行前必须先停止 chassis_bridge，否则串口被占用。
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None

from chassis.chassis_bridge import encode_navigation_frame


def main(argv=None):
    parser = argparse.ArgumentParser(description='MiniPC serial probe')
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='串口，建议使用 /dev/serial/by-id/...')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--seconds', type=float, default=20.0)
    # MiniPC 可能要求接近 chassis_bridge 的 50 Hz 帧率；20 Hz 可能被当成超时
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--forward', type=float, default=0.0)
    parser.add_argument('--lateral', type=float, default=0.0)
    parser.add_argument('--angular', type=float, default=0.0)
    parser.add_argument('--field', choices=('w', 'yaw', 'both'), default='w',
                        help='角速度写入 w_speed / yaw_speed / both')
    parser.add_argument('--no-write', action='store_true',
                        help='只读取，不发送任何帧')
    parser.add_argument('--echo-tx', action='store_true',
                        help='打印每一帧发出的 20 字节十六进制')
    parser.add_argument('--scan-mode', action='store_true',
                        help='scan_mod_type 置 1')
    parser.add_argument('--send-hex', default='',
                        help='直接重复发送这串十六进制，例如 "55 04 ..."；'
                             '用于回放同学/厂商抓到的帧')
    parser.add_argument('--startup-hex', default='',
                        help='启动时先发送的模式/使能帧，随后再发正常帧')
    parser.add_argument('--startup-repeat', type=int, default=5,
                        help='模式帧重复次数，默认 5')
    args = parser.parse_args(argv)

    if serial is None:
        print('pyserial 未安装：sudo apt install python3-serial')
        return 1

    print('port=%s baud=%d seconds=%.1f write=%s'
          % (args.port, args.baud, args.seconds, not args.no_write))
    print('forward=%.3f lateral=%.3f angular=%.3f field=%s scan_mode=%s'
          % (args.forward, args.lateral, args.angular, args.field,
             args.scan_mode))

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except Exception as exc:
        print('打开串口失败: %s' % exc)
        return 1

    yaw = args.angular if args.field in ('yaw', 'both') else 0.0
    w = args.angular if args.field in ('w', 'both') else 0.0

    if args.send_hex:
        try:
            frame = bytes.fromhex(args.send_hex)
        except ValueError as exc:
            print('--send-hex 格式错误: %s' % exc)
            ser.close()
            return 1
        if len(frame) < 2:
            print('--send-hex 至少需要 2 字节')
            ser.close()
            return 1
        # 退出时尽量发一帧“速度字段清零”的同类帧：保留帧头/帧尾，
        # 把中间字节清零。仅对 20 字节 VisionToGimbal 帧有确定意义。
        zero = bytearray(frame)
        for i in range(2, len(zero) - 1):
            zero[i] = 0
        zero = bytes(zero)
        print('send_hex=True len=%d' % len(frame))
    else:
        frame = encode_navigation_frame(
            args.forward, args.lateral, yaw, w, args.scan_mode)
        zero = encode_navigation_frame(0.0, 0.0, 0.0, 0.0, args.scan_mode)

    startup = None
    if args.startup_hex:
        try:
            startup = bytes.fromhex(args.startup_hex)
        except ValueError as exc:
            print('--startup-hex 格式错误: %s' % exc)
            ser.close()
            return 1
        print('startup_hex len=%d repeat=%d'
              % (len(startup), max(1, args.startup_repeat)))

    if startup is not None:
        for _ in range(max(1, args.startup_repeat)):
            ser.write(startup)
            ser.flush()
            time.sleep(0.02)
        time.sleep(0.1)

    rx_total = 0
    tx_count = 0
    deadline = time.monotonic() + max(0.1, args.seconds)
    period = 1.0 / max(1.0, args.rate)
    next_tx = time.monotonic()
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if not args.no_write and now >= next_tx:
                ser.write(frame)
                ser.flush()
                tx_count += 1
                if args.echo_tx:
                    print('TX %4d %s' % (tx_count, frame.hex(' ')))
                next_tx = now + period
            waiting = ser.in_waiting
            if waiting:
                data = ser.read(waiting)
                if data:
                    rx_total += len(data)
                    print('RX %4d bytes: %s' % (len(data), data.hex(' ')))
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not args.no_write:
                ser.write(zero)
                ser.flush()
            ser.close()
        except Exception:
            pass

    print('RX total: %d bytes' % rx_total)
    if rx_total == 0:
        print('没有收到任何反馈字节：MiniPC 没有在这条 CDC 上发送数据，'
              '或反馈需要额外请求/另一个接口。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
