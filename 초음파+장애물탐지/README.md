# 🚦 장애물 회피 및 주행 알고리즘 (Obstacle Avoidance Logic)

> **개요:** 전방의 초음파 센서(2개)가 장애물을 감지하면, 측면 IR 센서(2개)를 통해 안전한 경로를 탐색하고 회피 기동을 수행하는 로직입니다.

---

## 1. 센서 구성 및 역할 (Sensor Configuration)

| 센서 타입 | 수량 | 역할 (Role) | 변수명 예시 |
| :--- | :---: | :--- | :--- |
| **초음파 센서**<br>(HC-SR04) | 2 | **전방 거리 감지**<br>- 전방 장애물까지의 거리(cm) 측정<br>- 두 센서 중 하나라도 임계값 이하일 경우 감지로 판단 | `dist_L`, `dist_R` |
| **적외선 센서**<br>(IR-08H) | 2 | **좌우 측면 장애물 유무 판단**<br>- 근거리(측면) 장애물 유무 확인 (Digital Input 0/1) | `ir_left`, `ir_right` |

---

## 2. 감지 로직 흐름도 (Logic Flow)

알고리즘은 **전방 감시(Monitoring)** 상태를 유지하다가 조건 충족 시 **회피 기동(Avoidance)** 으로 전환됩니다.

### 📌 Step 1: 전방 거리 측정 (Monitoring)
1.  듀얼 초음파 센서(`Ultrasonic 1`, `Ultrasonic 2`)를 통해 전방 거리를 지속적으로 측정합니다.
2.  **판단 기준:** 두 센서 중 **최소값**이 **15cm 이하**인가?
    * `min(dist1, dist2) <= 15cm`

### 📌 Step 2: 측면 공간 탐색 (Scanning)
전방 장애물이 감지된 순간(`True`), 차량을 **일시 정지(Stop)** 하고 좌우 IR 센서 값을 읽습니다.

* **Left IR Check:** 좌측 장애물 유무 (`HIGH`: 장애물 없음 / `LOW`: 장애물 있음) *※ 센서 특성에 따라 다름*
* **Right IR Check:** 우측 장애물 유무

### 📌 Step 3: 주행 방향 결정 (Decision Making)
IR 센서 값의 조합에 따라 다음 행동을 결정합니다.

| Case | 전방 (Ultrasonic) | 좌측 (IR Left) | 우측 (IR Right) | **행동 (Action)** |
| :---: | :---: | :---: | :---: | :--- |
| **1** | **15cm 이내** | **장애물 없음** | **장애물 없음** | **우회전 (기본값) 또는 랜덤 회전** |
| **2** | **15cm 이내** | **장애물 없음** | 장애물 있음 | **좌회전 (Left Turn)** |
| **3** | **15cm 이내** | 장애물 있음 | **장애물 없음** | **우회전 (Right Turn)** |
| **4** | **15cm 이내** | 장애물 있음 | 장애물 있음 | **후진 후 180도 회전 (U-Turn)** |
| **5** | 15cm 초과 | (무시) | (무시) | **직진 (Go Straight)** |

---
## 3. 듀얼 초음파 센서 측정 테스트
<img width="634" height="529" alt="image" src="https://github.com/user-attachments/assets/042074c4-e1fe-4fb5-9f0c-cd87493d230f" />

        Motor_Forward();
    }
}
