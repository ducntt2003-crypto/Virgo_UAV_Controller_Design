# Virgo_UAV: Omnidirectional Morphing Multi-Rotor UAV
## Mô Hình Toán Học, Động Lực Học và Ma Trận Phân Bổ Lực Dạng Đóng (Closed-Form Control Allocation)

---

## 1. Cấu Trúc Hình Học & Hệ Toạ Độ (Kinematic Configuration)

Virgo OMAV sở hữu kết cấu khung chữ H (H-Frame) với 4 cụm cánh tay đặt tại các đỉnh đối xứng qua trọng tâm ($CoM$):
$$\mathbf{p}_0 = [+L_x, -L_y, 0]^T \quad (\text{Front-Right})$$
$$\mathbf{p}_1 = [-L_x, -L_y, 0]^T \quad (\text{Rear-Right})$$
$$\mathbf{p}_2 = [-L_x, +L_y, 0]^T \quad (\text{Rear-Left})$$
$$\mathbf{p}_3 = [+L_x, +L_y, 0]^T \quad (\text{Front-Left})$$

Trong đó thông số kích thước thực tế của Virgo OMAV:
- Dầm ngang trung tâm: $2 L_y = 0.44\,\text{m} \implies L_y = 0.22\,\text{m}$
- Dầm dọc hai bên: $2 L_x = 0.56\,\text{m} \implies L_x = 0.28\,\text{m}$
- Bán kính hiệu dụng $r = \sqrt{L_x^2 + L_y^2} \approx 0.356\,\text{m}$

### 1.1 Cụm động cơ đồng trục triệt tiêu mô-men xoắn (Coaxial Rotor Pods)
Mỗi cụm cánh tay trang bị 2 động cơ không chổi than quay ngược chiều:
- Cánh trên (Upper, CCW): Lực đẩy $f_{u,i} = c_f \omega_{u,i}^2$, mô-men cản $\tau_{u,i} = -c_\tau \omega_{u,i}^2$.
- Cánh dưới (Lower, CW): Lực đẩy $f_{l,i} = c_f \omega_{l,i}^2$, mô-men cản $\tau_{l,i} = +c_\tau \omega_{l,i}^2$.
- Khi tốc độ cân bằng $\omega_{u,i} = \omega_{l,i}$, tổng lực đẩy pod $i$ là $T_i = 2 c_f \omega_i^2$ và mô-men xoắn tự sinh triệt tiêu:
  $$\tau_{\text{net},i} = \tau_{u,i} + \tau_{l,i} = 0$$

### 1.2 Cơ cấu Gimbal Servo kép 2 bậc tự do (2-DOF Gimbal Mechanism)
Mỗi pod được gắn trên cụm gimbal 2 servo:
- Servo 1: Góc nghiêng Pitch $\alpha_i \in [-\pi/2, \pi/2]$ quanh trục $Y_B$.
- Servo 2: Góc nghiêng Roll $\beta_i \in [-\pi/2, \pi/2]$ quanh trục $X_B$.

Vector đơn vị hướng lực đẩy của pod $i$ trong hệ toạ độ thân $\mathcal{F}_B$ (theo chuẩn FLU):
$$\mathbf{n}_i(\alpha_i, \beta_i) = \begin{bmatrix} \sin\alpha_i \cos\beta_i \\ -\sin\beta_i \\ \cos\alpha_i \cos\beta_i \end{bmatrix}$$
Lực đẩy vector tại cụm pod $i$:
$$\mathbf{t}_i = T_i \mathbf{n}_i = \begin{bmatrix} t_{i,x} \\ t_{i,y} \\ t_{i,z} \end{bmatrix}$$

---

## 2. Phương Trình Động Lực Học 6 Bậc Tự Do (Newton-Euler 6-DOF)

Hệ phương trình vi phân phi tuyến mô tả chuyển động của UAV trong không gian:
$$\begin{aligned}
m \ddot{\mathbf{p}}_W &= -m g \mathbf{e}_3 + \mathbf{R}_B^W \mathbf{f}_B \\
\mathbf{J} \dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\mathbf{J} \boldsymbol{\omega}_B) &= \boldsymbol{\tau}_B
\end{aligned}$$

Trong đó:
- $m$: Tổng khối lượng UAV ($2.80\,\text{kg}$).
- $\mathbf{J} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$: Ma trận quán tính chính.
- $\mathbf{R}_B^W \in SO(3)$: Ma trận quay từ hệ thân $\mathcal{F}_B$ sang hệ quy chiếu thế giới $\mathcal{F}_W$.
- $\mathbf{f}_B = \sum_{i=0}^3 \mathbf{t}_i$: Tổng lực đẩy trong hệ thân $\mathcal{F}_B$.
- $\boldsymbol{\tau}_B = \sum_{i=0}^3 (\mathbf{p}_i \times \mathbf{t}_i) + \boldsymbol{\tau}_{\text{coaxial}}$: Tổng mô-men tác dụng lên trọng tâm.

---

## 3. Thuật Toán Phân Bổ Lực Giải Tích Dạng Đóng (Closed-Form Control Allocation)

Từ vector lực mong muốn $\mathbf{f}_d = [f_x, f_y, f_z]^T$ và mô-men mong muốn $\boldsymbol{\tau}_d = [\tau_x, \tau_y, \tau_z]^T$, giải bài toán tối ưu hoá năng lượng bình phương cực tiểu:
$$\min_{\mathbf{t}_0, \dots, \mathbf{t}_3} \sum_{i=0}^3 \|\mathbf{t}_i\|^2 \quad \text{thỏa mãn} \quad \begin{cases} \sum \mathbf{t}_i = \mathbf{f}_d \\ \sum (\mathbf{p}_i \times \mathbf{t}_i) = \boldsymbol{\tau}_d \end{cases}$$

Nghiệm giải tích dạng đóng chính xác cho khung chữ H ($L_x \ne L_y$):

$$\begin{aligned}
\mathbf{t}_0 &= \frac{1}{4} \mathbf{f}_d + \begin{bmatrix} 0 \\ 0 \\ +\frac{\tau_x}{4 L_y} - \frac{\tau_y}{4 L_x} \end{bmatrix} \\
\mathbf{t}_1 &= \frac{1}{4} \mathbf{f}_d + \begin{bmatrix} 0 \\ 0 \\ +\frac{\tau_x}{4 L_y} + \frac{\tau_y}{4 L_x} \end{bmatrix} \\
\mathbf{t}_2 &= \frac{1}{4} \mathbf{f}_d + \begin{bmatrix} 0 \\ 0 \\ -\frac{\tau_x}{4 L_y} + \frac{\tau_y}{4 L_x} \end{bmatrix} \\
\mathbf{t}_3 &= \frac{1}{4} \mathbf{f}_d + \begin{bmatrix} 0 \\ 0 \\ -\frac{\tau_x}{4 L_y} - \frac{\tau_y}{4 L_x} \end{bmatrix}
\end{aligned}$$

### Trích xuất góc Servo và Lực đẩy Động cơ:
$$\alpha_i = \text{atan2}(t_{i,x},\, t_{i,z})$$
$$\beta_i = \text{atan2}(-t_{i,y},\, \sqrt{t_{i,x}^2 + t_{i,z}^2})$$
$$T_i = \sqrt{t_{i,x}^2 + t_{i,y}^2 + t_{i,z}^2}$$

---

## 4. Các Đột Phá Thiết Kế Điều Khiển Của Virgo_UAV_PID

### 4.1 Chiếu Lực Theo Góc Hướng Thực Tế (Actual-Heading Force Projection)
Trong firmware gốc của multicopter nghiêng thân, lực thế giới $\mathbf{f}_W$ được chiếu vào thân qua ma trận quay góc đặt mong muốn:
$$\mathbf{f}_B = \mathbf{R}_d^T \mathbf{f}_W$$
Khi UAV thực hiện quay Yaw nhanh ($\Delta \psi$), tồn tại độ trễ góc hướng $\mathbf{R}_{\text{actual}} \ne \mathbf{R}_d$, dẫn đến thành phần lực thực tế bị sai lệch góc:
$$\mathbf{f}_{W, \text{actual}} = \mathbf{R}_{\text{actual}} \mathbf{R}_d^T \mathbf{f}_W \ne \mathbf{f}_W$$
Gây trôi dạt toạ độ $X, Y$ nghiêm trọng.

**Giải pháp của Virgo_UAV_PID:**
Chiếu trực tiếp lực thông qua góc hướng đo đạc từ cảm biến trạng thái thực tế `states.yaw`:
$$\mathbf{R}_{\text{actual}} = \mathbf{R}(\phi_{\text{body}}, \theta_{\text{body}}, \psi_{\text{actual}})$$
$$\mathbf{f}_B = \mathbf{R}_{\text{actual}}^T \mathbf{f}_W$$
Triệt tiêu hoàn toàn sự ghép kênh giữa góc xoay Yaw và chuyển động tịnh tiến ngang.

### 4.2 Điều Khiển Yaw Bằng Phản Lực Đồng Trục (Coaxial Differential Reaction Torque)
Thay vì dùng servo cơ khí (độ trễ lớn $\sim 150\,\text{ms}$) để tạo mô-men Yaw, Virgo tận dụng cặp cánh quạt đồng trục trên dưới:
$$\Delta \text{throttle}_i = \pm 0.25 \cdot \tau_z$$
Thời gian đáp ứng của động cơ điện đạt $\sim 15\,\text{ms}$ (nhanh gấp 10 lần servo), giúp giữ vững góc Yaw tuyệt đối và loại bỏ hoàn toàn nhiễu giật servo.

### 4.3 Khóa Cứng Servo Khi Tiếp Đất & Cất Cánh (Gated Ground-Lock)
Khi $Z < 0.12\,\text{m}$, toàn bộ servo được khóa thẳng đứng $\alpha_i = 0^\circ, \beta_i = 0^\circ$. Khi $Z \ge 0.12\,\text{m}$, hệ thống mở khoá toàn bộ 6 bậc tự do, loại bỏ hoàn toàn nguy cơ quẹt cánh xuống đất và triệt tiêu độ trôi toạ độ khi hạ cánh.
