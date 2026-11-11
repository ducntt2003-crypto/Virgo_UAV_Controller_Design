import xml.etree.ElementTree as ET

def generate_sdf():
    sdf_content = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="virgo">
    <pose>0 0 0.35 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>

    <!-- ==================== BASE LINK: SQUARE SYMMETRIC FUSELAGE & PROBE ==================== -->
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>1.10</mass>
        <inertia>
          <ixx>0.025</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.025</iyy>
          <iyz>0</iyz>
          <izz>0.045</izz>
        </inertia>
      </inertial>

      <!-- Central Fuselage (Avionics & Battery Core at CoM) -->
      <collision name="fuselage_col">
        <geometry><box><size>0.18 0.18 0.09</size></box></geometry>
        <surface>
          <contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact>
          <friction><ode/></friction>
        </surface>
      </collision>
      <visual name="fuselage_vis">
        <geometry><box><size>0.18 0.18 0.09</size></box></geometry>
        <material>
          <ambient>0.10 0.15 0.22 1</ambient>
          <diffuse>0.15 0.25 0.38 1</diffuse>
          <specular>0.8 0.8 0.9 1</specular>
        </material>
      </visual>

      <!-- Inspection End-Effector Tool (Forward Inspection Probe along +X) -->
      <visual name="probe_vis">
        <pose>0.24 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.24</length></cylinder></geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
        </material>
      </visual>
      <visual name="probe_tip_vis">
        <pose>0.36 0 0 0 0 0</pose>
        <geometry><sphere><radius>0.014</radius></sphere></geometry>
        <material>
          <ambient>0.9 0.1 0.1 1</ambient>
          <diffuse>0.95 0.2 0.2 1</diffuse>
        </material>
      </visual>

      <!-- ==================== SQUARE FRAME CARBON STRUCTURE (Lx=0.25, Ly=0.25) ==================== -->
      <!-- 1. Central Cross X-Beam (along X from -0.25 to +0.25) -->
      <visual name="center_x_beam_vis">
        <pose>0 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.50</length></cylinder></geometry>
        <material><diffuse>0.15 0.15 0.15 1</diffuse></material>
      </visual>

      <!-- 2. Central Cross Y-Beam (along Y from -0.25 to +0.25) -->
      <visual name="center_y_beam_vis">
        <pose>0 0 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.50</length></cylinder></geometry>
        <material><diffuse>0.15 0.15 0.15 1</diffuse></material>
      </visual>

      <!-- 3. Left Longitudinal Boom (at Y = +0.25, along X from -0.25 to +0.25) -->
      <visual name="left_boom_vis">
        <pose>0 0.25 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.50</length></cylinder></geometry>
        <material><diffuse>0.1 0.3 0.6 1</diffuse></material>
      </visual>

      <!-- 4. Right Longitudinal Boom (at Y = -0.25, along X from -0.25 to +0.25) -->
      <visual name="right_boom_vis">
        <pose>0 -0.25 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.50</length></cylinder></geometry>
        <material><diffuse>0.1 0.3 0.6 1</diffuse></material>
      </visual>

      <!-- 5. Front Transverse Boom (at X = +0.25, along Y from -0.25 to +0.25) -->
      <visual name="front_boom_vis">
        <pose>0.25 0 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.50</length></cylinder></geometry>
        <material><diffuse>0.1 0.3 0.6 1</diffuse></material>
      </visual>

      <!-- 6. Back Transverse Boom (at X = -0.25, along Y from -0.25 to +0.25) -->
      <visual name="back_boom_vis">
        <pose>-0.25 0 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.50</length></cylinder></geometry>
        <material><diffuse>0.1 0.3 0.6 1</diffuse></material>
      </visual>

      <!-- 4 Landing Gear Legs -->
      <visual name="leg_vis_0"><pose>0.22 0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <collision name="leg_col_0"><pose>0.22 0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>

      <visual name="leg_vis_1"><pose>-0.22 0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <collision name="leg_col_1"><pose>-0.22 0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>

      <visual name="leg_vis_2"><pose>-0.22 -0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <collision name="leg_col_2"><pose>-0.22 -0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>

      <visual name="leg_vis_3"><pose>0.22 -0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <collision name="leg_col_3"><pose>0.22 -0.22 -0.15 0 0 0</pose><geometry><cylinder><radius>0.008</radius><length>0.30</length></cylinder></geometry><surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>

      <!-- Autopilot Sensors -->
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on><update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.00087</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>0.00087</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>0.00087</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0064</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0064</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0064</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on><update_rate>50</update_rate>
        <air_pressure><pressure><noise type="gaussian"><mean>0</mean><stddev>3</stddev></noise></pressure></air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on><update_rate>100</update_rate>
        <magnetometer>
          <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
        </magnetometer>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on><update_rate>30</update_rate>
      </sensor>
    </link>
"""

    pods = [
        (0, 0.25, 0.25, 0, 1),    # Arm 0: Front-Left (+Lx, +Ly)
        (1, -0.25, 0.25, 2, 3),   # Arm 1: Back-Left (-Lx, +Ly)
        (2, -0.25, -0.25, 4, 5),  # Arm 2: Back-Right (-Lx, -Ly)
        (3, 0.25, -0.25, 6, 7),   # Arm 3: Front-Right (+Lx, -Ly)
    ]

    for idx, px, py, m_top, m_bot in pods:
        sdf_content += f"""
    <!-- ================= POD {idx}: ({px:+.2f}, {py:+.2f}) ================= -->
    <link name="gimbal_pitch_link_{idx}">
      <pose>{px} {py} 0 0 0 0</pose>
      <inertial><mass>0.04</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz></inertia></inertial>
      <visual name="gp_vis_{idx}"><geometry><box><size>0.03 0.04 0.03</size></box></geometry><material><diffuse>0.2 0.4 0.7 1</diffuse></material></visual>
    </link>
    <joint name="servo_pitch_joint_{idx}" type="revolute">
      <parent>base_link</parent><child>gimbal_pitch_link_{idx}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1.5708</lower><upper>1.5708</upper><effort>10</effort><velocity>10</velocity></limit><dynamics><damping>0.05</damping></dynamics></axis>
    </joint>

    <link name="rotor_pod_link_{idx}">
      <pose>{px} {py} 0 0 0 0</pose>
      <inertial><mass>0.08</mass><inertia><ixx>3e-4</ixx><iyy>3e-4</iyy><izz>4e-4</izz></inertia></inertial>
      <!-- Pod core motor mount -->
      <visual name="pod_vis_{idx}"><geometry><cylinder><radius>0.02</radius><length>0.06</length></cylinder></geometry><material><diffuse>0.1 0.6 0.8 1</diffuse></material></visual>
      <!-- Duct Ring (Orange Propeller Guard as shown in MorphEUS Fig. 2) -->
      <visual name="duct_vis_{idx}">
        <pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>0.12</radius><length>0.04</length></cylinder></geometry>
        <material>
          <ambient>0.9 0.4 0.1 1</ambient>
          <diffuse>0.95 0.45 0.12 0.85</diffuse>
          <specular>0.6 0.6 0.6 1</specular>
        </material>
      </visual>
      <collision name="duct_col_{idx}">
        <geometry><cylinder><radius>0.12</radius><length>0.04</length></cylinder></geometry>
      </collision>
    </link>
    <joint name="servo_roll_joint_{idx}" type="revolute">
      <parent>gimbal_pitch_link_{idx}</parent><child>rotor_pod_link_{idx}</child>
      <axis><xyz>1 0 0</xyz><limit><lower>-1.5708</lower><upper>1.5708</upper><effort>10</effort><velocity>10</velocity></limit><dynamics><damping>0.05</damping></dynamics></axis>
    </joint>

    <!-- Upper Rotor (M{m_top}: CCW) -->
    <link name="rotor_{m_top}">
      <pose>{px} {py} 0.035 0 0 0</pose>
      <inertial><mass>0.015</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>2e-5</izz></inertia></inertial>
      <visual name="r{m_top}_vis">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8 0.8 0.8</scale>
            <uri>model://virgo/meshes/iris_prop_ccw.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.0 0.25 0.85 1</ambient>
          <diffuse>0.1 0.45 0.95 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <visual name="r{m_top}_hub_vis">
        <pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>0.014</radius><length>0.018</length></cylinder></geometry>
        <material><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
    <joint name="rotor_{m_top}_joint" type="revolute">
      <parent>rotor_pod_link_{idx}</parent><child>rotor_{m_top}</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>

    <!-- Lower Rotor (M{m_bot}: CW) -->
    <link name="rotor_{m_bot}">
      <pose>{px} {py} -0.035 0 0 0</pose>
      <inertial><mass>0.015</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>2e-5</izz></inertia></inertial>
      <visual name="r{m_bot}_vis">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8 0.8 0.8</scale>
            <uri>model://virgo/meshes/iris_prop_cw.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.85 0.15 0.1 1</ambient>
          <diffuse>0.95 0.25 0.15 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <visual name="r{m_bot}_hub_vis">
        <pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>0.014</radius><length>0.018</length></cylinder></geometry>
        <material><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
    <joint name="rotor_{m_bot}_joint" type="revolute">
      <parent>rotor_pod_link_{idx}</parent><child>rotor_{m_bot}</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>
"""

    sdf_content += """
    <!-- ==================== 8 MULTICOPTER MOTOR PLUGINS ==================== -->
"""
    for m in range(8):
        ccw = (m % 2 == 0)
        direction = 'ccw' if ccw else 'cw'
        sdf_content += f"""    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{m}_joint</jointName><linkName>rotor_{m}</linkName><turningDirection>{direction}</turningDirection>
      <timeConstantUp>0.015</timeConstantUp><timeConstantDown>0.02</timeConstantDown><maxRotVelocity>1200</maxRotVelocity>
      <motorConstant>8.548e-06</motorConstant><momentConstant>0.06</momentConstant>
      <rotorDragCoefficient>0.0001</rotorDragCoefficient><rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <commandSubTopic>command/motor_speed</commandSubTopic><actuator_number>{m}</actuator_number>
      <motorSpeedPubTopic>motor_speed/{m}</motorSpeedPubTopic><rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim><motorType>velocity</motorType>
    </plugin>
"""

    sdf_content += """
    <!-- ==================== 8 GIMBAL SERVO CONTROLLERS ==================== -->
"""
    for i in range(4):
        p_ch = 2 * i
        r_ch = 2 * i + 1
        sdf_content += f"""    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>servo_pitch_joint_{i}</joint_name><sub_topic>servo_{p_ch}</sub_topic><p_gain>45.0</p_gain><d_gain>1.2</d_gain><i_gain>0.0</i_gain><cmd_max>8.0</cmd_max><cmd_min>-8.0</cmd_min>
    </plugin>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>servo_roll_joint_{i}</joint_name><sub_topic>servo_{r_ch}</sub_topic><p_gain>45.0</p_gain><d_gain>1.2</d_gain><i_gain>0.0</i_gain><cmd_max>8.0</cmd_max><cmd_min>-8.0</cmd_min>
    </plugin>
"""

    sdf_content += """  </model>
</sdf>
"""

    # Validate XML
    ET.fromstring(sdf_content)
    print("XML parsed and validated successfully!")

    paths = [
        "/home/duc/PX4/PX4-Autopilot/Tools/simulation/gz/models/virgo/model.sdf",
        "/home/duc/Omini_UAV/Virgo_UAV_PID_V2/models/virgo/model.sdf"
    ]
    for p in paths:
        with open(p, "w") as f:
            f.write(sdf_content)
        print(f"Written: {p}")

if __name__ == "__main__":
    generate_sdf()
