# -*- coding: utf-8 -*-
# 平衡球 6.0 - UDP 发送端 (MaixCAM2)
# 职责: YOLO 检测球位置 + 触屏设定目标 + UDP 广播给 ESP32
# 电机控制已移到 ESP32-S3 (Emm42_V5 串口指令), 本脚本不再驱动 GPIO
#
# 前提: MaixCAM2 已在系统设置里连上手机热点(和 ESP32、电脑同一热点)
#
# UDP 协议 (ASCII, 每帧一条, 广播到 255.255.255.255:5005):
#   有球:  B,<seq>,<ball_cm>,<target_cm>\n    例: B,1024,3.250,0.00
#   无球:  N,<seq>\n
#   电脑上抓包调试: python -c "import socket;s=socket.socket(2,2);s.bind(('',5005));
#                   [print(s.recvfrom(64)[0]) for _ in iter(int,1)]"

from maix import app, camera, display, image, nn, time, touchscreen
import socket
import os

MODEL_FILENAME = "model_312918.mud"
BALL_LABELS = ("gq", "ball", "gangqiu")

CONF_TH = 0.15
IOU_TH = 0.45

UDP_PORT = 5005
UDP_ADDR = "255.255.255.255"   # 广播: ESP32 和电脑都能收, 无需知道对方 IP

FILTER_ALPHA = 0.65            # 位置滤波(球速估计在 ESP32 端做)

# ---- 赛题第3问: O -> +5 -> 折返 -> -5 稳定, 总时长<=5s ----
T3_TARGET_A = 5.0
T3_TARGET_B = -5.0
T3_ARRIVE_CM = 4.7    # 球越过该位置视为"到达+5"(留少量提前量.
                         # 4.5时球仍只到4.5左右就折返, 再收窄到4.8)
T3_ARRIVE_HOLD_MS = 100  # 越过需保持的时间, 防检测毛刺误触发折返
T3_DONE_TOL_CM = 0.5     # 稳定判定: 距-5误差
T3_DONE_HOLD_MS = 500    # 在容差内保持该时长视为完成, 计时定格

ALLOW_BAND_HEIGHT_RATIO = 0.25
ALLOW_BAND_CUT_TOP_RATIO = 0.35
ALLOW_BAND_CUT_BOTTOM_RATIO = -0.20
RIGHT_CENTER_LIMIT_RATIO = 0.95

LOG_ENABLE = True              # 本地仍记一份位置日志, 方便离线核对
LOG_DIR = "/root/logs"
LOG_FLUSH_EVERY = 20


def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def find_model():
    candidates = (
        "/root/models/" + MODEL_FILENAME,
        "/root/models/maixhub/312918/" + MODEL_FILENAME,
        "/root/my_model/gangbest_model/" + MODEL_FILENAME,
        "/root/" + MODEL_FILENAME,
        MODEL_FILENAME,
    )
    for path in candidates:
        if os.path.isfile(path):
            return path
    for root, dirs, files in os.walk("/root"):
        if MODEL_FILENAME in files:
            return os.path.join(root, MODEL_FILENAME)
    raise RuntimeError("model not found")


def open_log_file():
    if not LOG_ENABLE:
        return None
    try:
        os.makedirs(LOG_DIR)
    except Exception:
        pass
    path = "%s/udp_send6_%d.csv" % (LOG_DIR, time.ticks_ms())
    f = open(path, "w")
    f.write("time_ms,seq,state,ball_cm,filtered_cm,target_cm,score\n")
    print("log file:", path)
    return f


def draw_start_screen(img):
    w = img.width()
    h = img.height()
    btn_w = 340
    btn_h = 140
    x = (w - btn_w) // 2
    y = (h - btn_h) // 2
    img.draw_rect(0, 0, w, h, color=image.COLOR_BLACK, thickness=-1)
    img.draw_rect(x, y, btn_w, btn_h, color=image.COLOR_GREEN, thickness=4)
    img.draw_string(x + 124, y + 60, "START", color=image.COLOR_GREEN)
    img.draw_string(18, h - 38, "Touch to start UDP sending", color=image.COLOR_WHITE)


def draw_allowed_area_and_grid(img):
    w = img.width()
    h = img.height()
    center_x = w // 2
    right_limit_x = int(w * RIGHT_CENTER_LIMIT_RATIO)
    right_limit_x = max(center_x + 1, min(w - 1, right_limit_x))
    center_y = h // 2
    base_band_h = int(h * ALLOW_BAND_HEIGHT_RATIO)
    base_band_top = center_y - base_band_h // 2
    base_band_bottom = center_y + base_band_h // 2
    band_top = base_band_top + int(base_band_h * ALLOW_BAND_CUT_TOP_RATIO)
    band_bottom = base_band_bottom - int(base_band_h * ALLOW_BAND_CUT_BOTTOM_RATIO)
    band_top = max(0, band_top)
    band_bottom = min(h - 1, band_bottom)
    band_h = band_bottom - band_top
    left_step = center_x / 12.0
    right_step = (right_limit_x - center_x) / 12.0

    img.draw_rect(0, band_top, w, band_h, color=image.COLOR_WHITE, thickness=1)
    for index in range(-12, 13):
        if index < 0:
            x = int(round(center_x + index * left_step))
        elif index > 0:
            x = int(round(center_x + index * right_step))
        else:
            x = center_x
        if x < 0 or x >= w:
            continue
        color = image.COLOR_BLUE if index == 0 else image.COLOR_WHITE
        img.draw_line(x, band_top, x, band_bottom, color=color)

    return band_top, band_bottom, left_step, right_step, center_x


def draw_top_menu(img, menu_val, target_cm):
    # 顶部任意目标菜单: [-] [当前选值] [+] [GO], 1cm步进, -10~+10
    w = img.width()
    bar_h = 46
    zone_w = w // 4
    labels = ("-", "%+d" % menu_val, "+", "GO")
    for i in range(4):
        x = i * zone_w
        if i == 1:
            color = image.COLOR_WHITE
        elif i == 3 and abs(target_cm - menu_val) < 0.01:
            color = image.COLOR_GREEN   # 当前目标就是选值
        else:
            color = image.COLOR_YELLOW
        img.draw_rect(x, 0, zone_w - 2, bar_h - 2, color=color, thickness=2)
        img.draw_string(x + zone_w // 2 - 10, 14, labels[i], color=color)


def draw_target_buttons(img, target_cm, task3_phase):
    w = img.width()
    h = img.height()
    zone_h = 46
    top = h - zone_h
    zone_w = w // 2
    labels = ("0", "T3")
    for i in range(2):
        x = i * zone_w
        active = (task3_phase == 0) if i == 0 else (task3_phase > 0)
        color = image.COLOR_YELLOW if active else image.COLOR_GRAY
        img.draw_rect(x, top, zone_w - 2, zone_h - 2, color=color, thickness=2)
        img.draw_string(x + zone_w // 2 - 10, top + 14, labels[i], color=color)


def is_in_allowed_band(obj, band_top, band_bottom):
    center_y = obj.y + obj.h // 2
    return band_top <= center_y <= band_bottom


def position_cm(dx, left_step, right_step):
    step = right_step if dx >= 0 else left_step
    if step <= 0:
        return 0.0
    return clamp(dx / step, -12.0, 12.0)


model_path = find_model()
print("model path:", model_path)

detector = nn.YOLOv5(model=model_path, dual_buff=True)
cam = camera.Camera(
    detector.input_width(),
    detector.input_height(),
    detector.input_format()
)
disp = display.Display()
ts = touchscreen.TouchScreen()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

started = False
target_cm = 0.0
menu_val = 0            # 顶部菜单选定的任意目标(整数cm)
seq = 0
last_touch_ms = 0
task3_phase = 0       # 0=手动模式 1=去+5 2=去-5 3=完成(定格显示)
task3_t0 = 0
task3_cross_ms = 0
task3_done_ms = 0
task3_elapsed = 0.0
log_file = None
log_rows = 0
filtered_ball_cm = 0.0
have_ball_history = False

while not app.need_exit():
    img = cam.read()
    now_ms = time.ticks_ms()

    if not started:
        draw_start_screen(img)
        touch_x, touch_y, pressed = ts.read()
        if pressed:
            # 触摸坐标是屏幕分辨率, 必须映射回图像坐标
            touch_x, touch_y = image.resize_map_pos_reverse(
                img.width(), img.height(), disp.width(), disp.height(),
                image.Fit.FIT_CONTAIN, touch_x, touch_y)[:2]
        if pressed and touch_y >= img.height() // 4:
            started = True
            log_file = open_log_file()
            have_ball_history = False
            print("START UDP sending ->", UDP_ADDR, UDP_PORT)
            time.sleep_ms(300)
            continue
        disp.show(img)
        continue

    # 触屏切目标 / 启动第3问流程
    touch_x, touch_y, pressed = ts.read()
    if pressed and now_ms - last_touch_ms > 400:
        # 屏幕坐标 -> 图像坐标 (屏幕552x368/640x480, 图像是模型输入尺寸)
        touch_x, touch_y = image.resize_map_pos_reverse(
            img.width(), img.height(), disp.width(), disp.height(),
            image.Fit.FIT_CONTAIN, touch_x, touch_y)[:2]
        if touch_y < 46:
            # 顶部菜单: [-] [值] [+] [GO]
            zone = min(3, touch_x * 4 // img.width())
            last_touch_ms = now_ms
            if zone == 0:
                menu_val = max(-10, menu_val - 1)
            elif zone == 2:
                menu_val = min(10, menu_val + 1)
            elif zone == 3:
                target_cm = float(menu_val)
                task3_phase = 0
                print("menu target ->", menu_val)
        elif touch_y >= img.height() - 46:
            zone = min(1, touch_x * 2 // img.width())
            last_touch_ms = now_ms
            if zone == 0:
                target_cm = 0.0
                task3_phase = 0
                print("target -> 0")
            else:
                task3_phase = 1
                task3_t0 = now_ms
                task3_cross_ms = 0
                task3_done_ms = 0
                target_cm = T3_TARGET_A
                print("T3 START: go +5")

    objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)
    band_top, band_bottom, left_step, right_step, grid_center_x = draw_allowed_area_and_grid(img)
    draw_target_buttons(img, target_cm, task3_phase)
    draw_top_menu(img, menu_val, target_cm)

    best_ball = None
    for obj in objs:
        label = detector.labels[obj.class_id].lower()
        if label not in BALL_LABELS:
            continue
        if not is_in_allowed_band(obj, band_top, band_bottom):
            continue
        if best_ball is None or obj.score > best_ball.score:
            best_ball = obj

    seq += 1
    if best_ball:
        ball_cx = best_ball.x + best_ball.w // 2
        dx = ball_cx - grid_center_x
        ball_cm = position_cm(dx, left_step, right_step)
        if have_ball_history:
            filtered_ball_cm = FILTER_ALPHA * ball_cm + (1.0 - FILTER_ALPHA) * filtered_ball_cm
        else:
            filtered_ball_cm = ball_cm
            have_ball_history = True

        # ---- 第3问状态机 ----
        if task3_phase == 1:
            if filtered_ball_cm >= T3_ARRIVE_CM:
                if task3_cross_ms == 0:
                    task3_cross_ms = now_ms
                elif now_ms - task3_cross_ms >= T3_ARRIVE_HOLD_MS:
                    task3_phase = 2
                    target_cm = T3_TARGET_B
                    print("T3: reached +5, turn to -5")
            else:
                task3_cross_ms = 0
        elif task3_phase == 2:
            if abs(filtered_ball_cm - T3_TARGET_B) <= T3_DONE_TOL_CM:
                if task3_done_ms == 0:
                    task3_done_ms = now_ms
                elif now_ms - task3_done_ms >= T3_DONE_HOLD_MS:
                    task3_phase = 3
                    task3_elapsed = (task3_done_ms - task3_t0) / 1000.0
                    print("T3 DONE: %.2fs" % task3_elapsed)
            else:
                task3_done_ms = 0

        msg = "B,%d,%.3f,%.2f\n" % (seq, filtered_ball_cm, target_cm)
        state = "B"
        score = best_ball.score

        img.draw_rect(best_ball.x, best_ball.y, best_ball.w, best_ball.h,
                      color=image.COLOR_RED, thickness=2)
        img.draw_string(5, 50, "BALL x=%.2f tgt=%.1f" % (filtered_ball_cm, target_cm),
                        color=image.COLOR_GREEN)
        img.draw_string(5, 70, "seq=%d" % seq, color=image.COLOR_GREEN)
        if task3_phase in (1, 2):
            img.draw_string(5, 90, "T3 P%d t=%.2fs" % (task3_phase, (now_ms - task3_t0) / 1000.0),
                            color=image.COLOR_YELLOW)
        elif task3_phase == 3:
            img.draw_string(5, 90, "T3 DONE %.2fs" % task3_elapsed, color=image.COLOR_YELLOW)
    else:
        have_ball_history = False
        msg = "N,%d\n" % seq
        state = "N"
        ball_cm = 0.0
        score = 0.0
        img.draw_string(5, 50, "NO BALL", color=image.COLOR_RED)

    try:
        sock.sendto(msg.encode(), (UDP_ADDR, UDP_PORT))
    except Exception as e:
        img.draw_string(5, 110, "UDP ERR: %s" % e, color=image.COLOR_RED)

    if log_file:
        log_file.write("%d,%d,%s,%.3f,%.3f,%.2f,%.3f\n" % (
            now_ms, seq, state, ball_cm, filtered_ball_cm, target_cm, score))
        log_rows += 1
        if log_rows % LOG_FLUSH_EVERY == 0:
            log_file.flush()

    disp.show(img)
