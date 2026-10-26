# 导入所需第三方库
import cv2                  # OpenCV库：摄像头读取、画面渲染、窗口显示
import mediapipe as mp      # 谷歌MediaPipe手部识别AI模型，输出21个手部三维关键点
import pyautogui            # 系统自动化库，控制鼠标移动、单击、右键、拖拽、双击
import math                 # 数学库，计算两点欧氏距离，判断手指伸直/弯曲
from enum import IntEnum    # 整数枚举类，给每一种手势分配固定数字编码，方便判断

# 关闭pyautogui安全机制：鼠标移到屏幕角落不会直接终止程序
pyautogui.FAILSAFE = False
# 简化mediapipe工具类别名
mp_drawing = mp.solutions.drawing_utils  # 绘制手部21个关键点与骨架连线
mp_hands = mp.solutions.hands            # MediaPipe手部识别模型核心类

# ====================== 手势编码枚举类 ======================
# 编码规则：四根手指【食指、中指、无名指、小指】4位二进制
# 食指权重8(2^3)、中指4(2^2)、无名指2(2^1)、小指1(2^0)；手指伸直=1，弯曲=0
# 大拇指单独权重16，不参与四指二进制计算
# FIRST2=12 代表食指+中指同时伸出(8+4)，在此基础上细分两种衍生手势33、34
class Gest(IntEnum):
    FIST = 0                # 0000 四根手指全部弯曲：握拳
    PINKY = 1               # 0001 仅小指伸直
    RING = 2                # 0010 仅无名指伸直
    MID = 4                 # 0100 仅中指伸直（右键单击手势）
    LAST3 = 7               # 0111 中指+无名指+小指伸出，食指弯曲
    INDEX = 8               # 1000 仅食指伸直（移动光标手势）
    FIRST2 = 12             # 1100 食指+中指同时伸出（基础双指状态，用于二次细分）
    LAST4 = 15              # 1111 四根手指全部伸直
    THUMB = 16              # 仅大拇指伸出，其余四指弯曲
    PALM = 31               # 16+15=31 四根手指+大拇指全部张开，完整手掌（无操作）
    V_GEST = 33             # 衍生手势：食指中指大幅分开，标准V字（左键单击）
    TWO_FINGER_CLOSED = 34  # 衍生手势：食指中指紧贴并拢（双击手势）

# 左右手标记枚举
class HLabel(IntEnum):
    MINOR = 0   # 副手：左手，本程序未使用左手功能
    MAJOR = 1   # 主手：右手，所有鼠标操作仅由右手手势控制

# ====================== 手部识别处理类 ======================
# 作用：接收MediaPipe手部关键点，计算手指状态，防抖后输出稳定手势编号
class HandRecog:
    def __init__(self, label):
        # label：标记当前是左手(HLabel.MINOR)还是右手(HLabel.MAJOR)
        self.label = label
        self.finger = Gest.PALM         # 当前帧四根手指二进制原始编码
        self.pre_gest = Gest.PALM       # 上一帧识别到的手势编号
        self.ori_gest = Gest.PALM       # 防抖稳定后的最终有效手势（连续多帧相同才更新）
        self.frame_cnt = 0              # 当前手势连续稳定帧数，用于防抖防误触
        self.hand_res = None            # 存储MediaPipe返回的21个手部关键点数据

    # 更新当前帧的手部关键点数据
    def update(self, res):
        self.hand_res = res

    # 计算两个关键点之间的平面欧式距离（无正负）
    # pts: [点1索引, 点2索引]
    def dist(self, pts):
        # 取出两个关键点的x、y归一化坐标(0~1)
        x1 = self.hand_res.landmark[pts[0]].x
        y1 = self.hand_res.landmark[pts[0]].y
        x2 = self.hand_res.landmark[pts[1]].x
        y2 = self.hand_res.landmark[pts[1]].y
        # 欧氏距离公式 sqrt((x1-x2)² + (y1-y2)²)
        return math.hypot(x1 - x2, y1 - y2)

    # 带正负号距离：区分指尖在指关节上方/下方，判断手指是否伸直
    # 返回正数=指尖在关节上方（手指伸直）；负数=指尖在关节下方（手指弯曲）
    def sign_dist(self, pts):
        d = self.dist(pts)
        # 比较两点y坐标，y越小代表画面上方
        if self.hand_res.landmark[pts[0]].y < self.hand_res.landmark[pts[1]].y:
            return d
        else:
            return -d

    # 核心函数：计算四根手指的二进制编码，存入self.finger
    def set_finger(self):
        # 无手部数据直接退出
        if self.hand_res is None:
            return
        # 四根手指关键点配置：[指尖索引, 中关节索引, 掌根索引]
        # 顺序：食指、中指、无名指、小指
        points = [[8, 5, 0], [12, 9, 0], [16, 13, 0], [20, 17, 0]]
        self.finger = 0  # 初始化四指编码为0000
        for p in points:
            # 指尖到中关节距离 / 中关节到掌根距离
            r = self.sign_dist(p[:2]) / self.sign_dist(p[1:])
            # 左移一位，腾出最低位存放当前手指状态
            self.finger = self.finger << 1
            # 比值>0.5判定手指伸直，最低位置1；否则为0
            if r > 0.5:
                self.finger = self.finger | 1

    # 防抖+手势细分，返回稳定可用的手势编号
    def get_gest(self):
        # 无手部关键点，默认返回手掌（无操作）
        if self.hand_res is None:
            return Gest.PALM
        cur = Gest.PALM
        # 判断基础编码是否为食指+中指同时伸出(FIRST2=12)
        if self.finger == Gest.FIRST2:
            # 计算两指尖间距 / 两指根间距，判断开合程度
            r = self.dist([8, 12]) / self.dist([5, 9])
            if r > 1.7:
                # 手指张开距离大：标准V手势，左键单击
                cur = Gest.V_GEST
            else:
                # 手指间距近，再判断深度z轴是否贴近
                z_diff = abs(self.hand_res.landmark[8].z - self.hand_res.landmark[12].z)
                if z_diff < 0.1:
                    # 两根手指紧贴：双击手势
                    cur = Gest.TWO_FINGER_CLOSED
                else:
                    # 手指重叠错开：判定为仅中指伸出
                    cur = Gest.MID
        else:
            # 非双指同时伸出，直接使用原始四指编码
            cur = self.finger

        # ====================== 防抖逻辑 ======================
        # 当前手势和上一帧相同，计数+1；否则重置计数
        if cur == self.pre_gest:
            self.frame_cnt += 1
        else:
            self.frame_cnt = 0
        # 保存当前手势到上一帧缓存
        self.pre_gest = cur
        # 连续稳定超过4帧，才更新为有效手势，过滤手部抖动误识别
        if self.frame_cnt > 4:
            self.ori_gest = cur
        # 返回防抖后的最终手势
        return self.ori_gest

# ====================== 鼠标控制静态工具类 ======================
# 接收识别出的手势，执行对应的鼠标操作
class Ctrl:
    prev_hand = None    # 上一帧手掌中心坐标，用于光标平滑防抖
    grab_flag = False   # 拖拽标记：True=左键按住拖拽状态

    # 将手掌关键点坐标转为屏幕像素坐标，并做平滑滤波消除光标抖动
    @staticmethod
    def pos(hand):
        # 获取屏幕宽、高像素尺寸
        screen_w, screen_h = pyautogui.size()
        # 关键点9为手掌中心，将0~1归一化坐标映射到屏幕像素
        x = int(hand.landmark[9].x * screen_w)
        y = int(hand.landmark[9].y * screen_h)
        # 首次识别手掌，初始化上一帧坐标
        if Ctrl.prev_hand is None:
            Ctrl.prev_hand = (x, y)
        # 计算当前帧与上一帧的位移差
        dx = x - Ctrl.prev_hand[0]
        dy = y - Ctrl.prev_hand[1]
        # 更新上一帧坐标缓存
        Ctrl.prev_hand = (x, y)
        # 位移平方，判断手部移动幅度
        dist_sq = dx * dx + dy * dy
        # 动态平滑系数：小幅移动低灵敏度防抖动，大幅移动提升跟随速度
        if dist_sq <= 25:
            ratio = 0    # 微小抖动，光标完全不动
        elif dist_sq <= 900:
            ratio = 0.07 * math.sqrt(dist_sq)  # 小幅移动，低平滑倍率
        else:
            ratio = 2.1  # 大幅度挥手，高倍率快速移动光标
        # 获取鼠标当前真实屏幕坐标
        old_x, old_y = pyautogui.position()
        # 输出平滑后的光标坐标
        return old_x + dx * ratio, old_y + dy * ratio

    # 手势分发核心函数：根据手势执行对应鼠标操作
    @staticmethod
    def run(gest, hand):
        # 手掌张开时不计算坐标，置空
        if gest != Gest.PALM:
            mouse_x, mouse_y = Ctrl.pos(hand)
        else:
            mouse_x, mouse_y = None, None

        # 如果当前不是握拳拖拽状态，松开鼠标左键，防止拖拽卡死
        if gest != Gest.FIST and Ctrl.grab_flag:
            Ctrl.grab_flag = False
            pyautogui.mouseUp()

        # 1. 仅伸出食指：持续移动光标
        if gest == Gest.INDEX:
            pyautogui.moveTo(mouse_x, mouse_y, duration=0.1)
        # 2. V字双指分开：左键单击一次
        elif gest == Gest.V_GEST:
            pyautogui.click()
        # 3. 仅伸出中指：右键单击一次
        elif gest == Gest.MID:
            pyautogui.click(button='right')
        # 4. 食指中指紧贴并拢：鼠标双击
        elif gest == Gest.TWO_FINGER_CLOSED:
            pyautogui.doubleClick()
        # 5. 全握拳：按住左键，跟随手掌移动实现拖拽
        elif gest == Gest.FIST:
            # 首次握拳，按下左键
            if not Ctrl.grab_flag:
                Ctrl.grab_flag = True
                pyautogui.mouseDown()
            # 拖拽时持续更新光标位置
            pyautogui.moveTo(mouse_x, mouse_y, duration=0.1)

# ====================== 主程序调度类 ======================
# 负责摄像头初始化、画面循环、左右手区分、流程调度
class GestController:
    def __init__(self):
        # 打开默认摄像头(0号设备)
        self.cap = cv2.VideoCapture(0)
        # 获取摄像头原始画面宽、高像素
        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # 实例化右手、左手识别处理对象
        self.major = HandRecog(HLabel.MAJOR)    # 右手（主操作手）
        self.minor = HandRecog(HLabel.MINOR)    # 左手（本程序闲置无功能）

    # 区分画面中识别到的左手、右手，分别存入对应识别对象
    def classify(self, res):
        left_hand = None
        right_hand = None
        # 遍历所有识别到的手部结果（最多2只手）
        for idx, hand_land in enumerate(res.multi_hand_landmarks):
            # 将MediaPipe手部标签转为字典，读取Left/Right标识
            hand_info = MessageToDict(res.multi_handedness[idx])
            hand_label = hand_info['classification'][0]['label']
            if hand_label == 'Right':
                right_hand = hand_land
            else:
                left_hand = hand_land
        # 更新左右手关键点数据
        self.major.update(right_hand)
        self.minor.update(left_hand)

    # 程序主循环：持续读取摄像头、识别手势、执行鼠标操作、渲染画面
    def start(self):
        # 初始化手部AI模型：最多识别2只手，最低检测置信度0.5，跟踪置信度0.5
        with mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            # 摄像头正常打开则持续循环
            while self.cap.isOpened():
                # 读取一帧摄像头画面
                ok, img = self.cap.read()
                # 读取失败跳过当前帧
                if not ok:
                    continue
                # 画面水平镜像，符合人眼视觉习惯（左右不反转）
                img = cv2.flip(img, 1)
                # OpenCV默认BGR色彩，转为RGB送入MediaPipe模型
                rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                # 模型处理画面，输出手部识别结果
                out = hands.process(rgb_img)

                # 画面识别到手部关键点
                if out.multi_hand_landmarks:
                    # 区分左右手
                    self.classify(out)
                    # 计算左右手四指二进制编码
                    self.minor.set_finger()
                    self.major.set_finger()
                    # 获取右手防抖后的稳定手势
                    current_gest = self.major.get_gest()
                    # 根据手势执行鼠标操作（仅右手生效）
                    Ctrl.run(current_gest, self.major.hand_res)
                    # 在原图上绘制所有手部21个关键点+骨架连线
                    for hand_lm in out.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(img, hand_lm, mp_hands.HAND_CONNECTIONS)
                else:
                    # 画面无手，清空上一帧手掌坐标，光标停止跟随
                    Ctrl.prev_hand = None

                # 弹出预览窗口，显示带手部骨架的摄像头画面
                cv2.imshow('Gesture Ctrl', img)
                # 等待5ms按键输入，按下回车键(ASCII=13)跳出循环结束程序
                if cv2.waitKey(5) == 13:
                    break
        # 循环结束，释放摄像头硬件资源
        self.cap.release()
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()

# 程序入口：脚本直接运行时执行主程序
if __name__ == "__main__":
    # 延迟导入左右手解析依赖（仅运行时加载，减少启动开销）
    from google.protobuf.json_format import MessageToDict
    # 实例化控制器并启动主循环
    gc = GestController()
    gc.start()