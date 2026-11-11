# Virgo_UAV_PID - Hướng Dẫn Thực Hiện Các Bài Bay Đa Hướng (Mission Guide)

Tài liệu hướng dẫn chi tiết quy trình chạy mô phỏng, thực thi nhiệm vụ thông qua ROS 2 Humble và Python scripts, cùng quy trình trích xuất và phân tích dữ liệu bay cho UAV Đa hướng Biến hình **Virgo_UAV_PID**.

---

## 1. Các Chế Độ Bay Đa Hướng 6 Bậc Tự Do (6-DOF)

### 1.1 Quỹ đạo Square 25m Đa Trục Decoupled (Benchmark chính)
- **Mục tiêu:** Bay hoàn thành chu trình hình vuông cạnh $25\,\text{m}$ ở độ cao $1.5\,\text{m}$ kết hợp kiểm tra bám góc tư thế độc lập:
  - **Cạnh 1:** Nghiêng Roll $30^\circ$, Pitch $0^\circ$, Yaw $0^\circ$.
  - **Cạnh 2:** Nghiêng Pitch $30^\circ$, Roll $0^\circ$, Yaw $0^\circ$.
  - **Cạnh 3:** Xoay Yaw $30^\circ$ độc lập, Roll $0^\circ$, Pitch $0^\circ$.
  - **Cạnh 4:** Bay tịnh tiến thuần túy không nghiêng thân (Zero-Tilt Translation).
  - **Hạ cánh:** Tự động hạ cánh chính xác về gốc toạ độ $(0, 0, 0)$.
- **Script thực thi:** `scripts/test_square_decoupled_6dof.py`

### 1.2 Zero-Tilt Translation (Bay tịnh tiến ngang không nghiêng thân)
- **Mục tiêu:** Di chuyển theo quỹ đạo ngang ở độ cao ổn định trong khi thân máy bay duy trì tuyệt đối ở tư thế nằm ngang ($\text{roll} = 0^\circ, \text{pitch} = 0^\circ$). Toàn bộ gia tốc ngang được sinh ra bởi 8 servo nghiêng các cụm động cơ.
- **Script thực thi:** `scripts/test_straight_x_zero_tilt.py`

### 1.3 Continuous Contact Inspection (Kiểm tra bề mặt tiếp xúc liên tục)
- **Mục tiêu:** Áp sát bề mặt thẳng đứng (tường phẳng), duy trì thanh đo (end-effector probe) vuông góc với mặt tiếp xúc khi thân máy bay thay đổi góc chúc/nghiêng thân liên tục ($\text{pitch} = +15^\circ \to +30^\circ$).
- **Script thực thi:** `ros2 run virgo_missions virgo_mission_node --ros-args -p mission_type:=contact_inspection`

### 1.4 Flying in Constrained Narrow Spaces (Bay luồn lách qua không gian hẹp)
- **Mục tiêu:** Bay qua các khe hẹp bị giới hạn kích thước với góc nghiêng roll liên tục $\pm 25^\circ \to \pm 35^\circ$ để giảm diện tích tiết diện cản phía trước mà không làm sai lệch tâm quỹ đạo bay.
- **Script thực thi:** `ros2 run virgo_missions virgo_mission_node --ros-args -p mission_type:=narrow_spaces`

### 1.5 Continuous Visual Inspection (Kiểm tra trực quan với góc nhìn cố định)
- **Mục tiêu:** Bay quỹ đạo tròn bán kính $R = 1.5\,\text{m}$ xung quanh điểm quan sát trung tâm (Point of Interest - POI) với camera luôn chĩa vào tâm.
- **Script thực thi:** `ros2 run virgo_missions virgo_mission_node --ros-args -p mission_type:=visual_inspection`

---

## 2. Quy Trình Chạy Mô Phỏng & Thu Thập Dữ Liệu

### Bước 1: Khởi động Mô phỏng PX4 SITL + Gazebo Harmonic
Mở Terminal 1:
```bash
cd /home/duc/Omini_UAV/Virgo_UAV_PID/scripts
./run_virgo_sitl.sh
# Hoặc mở giao diện 3D Gazebo:
./run_virgo_sitl.sh --gui
```

### Bước 2: Chạy Nhiệm Vụ Bay

**Cách 1: Chạy Benchmark Square Decoupled (Khuyên dùng)**
```bash
cd /home/duc/Omini_UAV/Virgo_UAV_PID/scripts
python3 test_square_decoupled_6dof.py
```

**Cách 2: Chạy thông qua ROS 2 Humble**
```bash
source /opt/ros/humble/setup.bash
source /home/duc/Omini_UAV/Virgo_UAV_PID/ros2_ws/install/setup.bash
./scripts/run_ros2_mission.sh square
```

### Bước 3: Vẽ Đồ Thị & Phân Tích Telemetry
Script `scripts/plot_flight_data.py` tự động đọc file CSV trong `docs/flight_logs/` và xuất đồ thị 6 bảng tiêu chuẩn vào `docs/flight_plots/`:
```bash
cd /home/duc/Omini_UAV/Virgo_UAV_PID/scripts
python3 plot_flight_data.py ../docs/flight_logs/virgo_square_decoupled_xxxx.csv
```
