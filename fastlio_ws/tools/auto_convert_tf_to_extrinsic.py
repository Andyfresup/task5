#!/usr/bin/env python3
"""
自动化脚本：从ROS TF树提取body->livox_frame变换，转换为FAST-LIO extrinsic参数，
并自动更新mid360.yaml配置文件。
"""

import sys
import os

# 优先使用系统Python路径中的ROS库，而不是虚拟环境中的
# 从虚拟环境的sys.path中临时移除
original_path = sys.path.copy()
sys.path = [p for p in sys.path if '.venv' not in p and 'venv' not in p]
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import math
import subprocess
import re
import yaml

try:
    import rospy
    import tf
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
except ImportError as e:
    print(f"错误：无法导入ROS库: {e}")
    print("确保已运行: source /opt/ros/noetic/setup.bash")
    print("然后不要激活虚拟环境，直接运行此脚本")
    sys.exit(1)


def get_tf_transform(source_frame, target_frame, timeout=5.0):
    """
    获取source_frame到target_frame的变换。
    返回 (translation, quaternion) 或 (None, None) 如果失败
    """
    print(f"\n[*] 正在获取 {source_frame} -> {target_frame} 的变换...")
    
    listener = tf.TransformListener()
    
    # 等待TF可用
    start_time = rospy.Time.now()
    timeout_duration = rospy.Duration(timeout)
    
    while (rospy.Time.now() - start_time) < timeout_duration:
        try:
            listener.waitForTransform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            print(f"[✓] 成功获取变换")
            print(f"    Translation: [{trans[0]:.6f}, {trans[1]:.6f}, {trans[2]:.6f}]")
            print(f"    Quaternion: [{rot[0]:.6f}, {rot[1]:.6f}, {rot[2]:.6f}, {rot[3]:.6f}]")
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            pass
        
        rospy.sleep(0.1)
    
    print(f"[✗] 无法获取 {source_frame} -> {target_frame} 的变换")
    print(f"    请确保：")
    print(f"    1. FAST-LIO已启动（roslaunch FAST_LIO task5_fastlio.launch）")
    print(f"    2. TF tree已发布")
    return None, None


def quaternion_to_rpy(quat):
    """
    将四元数转换为欧拉角（RPY，单位：弧度）
    四元数格式: [qx, qy, qz, qw]
    返回: (roll, pitch, yaw) 单位为弧度
    """
    euler = euler_from_quaternion(quat)
    return euler


def rpy_to_rotation_matrix(roll, pitch, yaw):
    """
    将RPY角（弧度）转换为3x3旋转矩阵
    使用ZYX欧拉角约定
    """
    # Z-Y-X欧拉角顺序
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    
    # 旋转矩阵
    R = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ]
    return R


def flatten_matrix(matrix):
    """将3x3矩阵展平为9个元素的列表"""
    return [matrix[i][j] for i in range(3) for j in range(3)]


def update_yaml_config(extrinsic_T, extrinsic_R, yaml_file):
    """
    更新mid360.yaml配置文件中的extrinsic参数
    """
    print(f"\n[*] 正在更新配置文件: {yaml_file}")
    
    if not os.path.exists(yaml_file):
        print(f"[✗] 文件不存在: {yaml_file}")
        return False
    
    try:
        with open(yaml_file, 'r') as f:
            content = f.read()
        
        # 更新extrinsic_T
        # 找到 extrinsic_T: [ ... ] 并替换
        T_str = f"[ {extrinsic_T[0]:.6f}, {extrinsic_T[1]:.6f}, {extrinsic_T[2]:.6f} ]"
        content = re.sub(
            r'extrinsic_T:\s*\[.*?\]',
            f'extrinsic_T: {T_str}',
            content
        )
        
        # 更新extrinsic_R
        # 找到 extrinsic_R: [ ... ] 并替换
        R_str = f"[ {extrinsic_R[0]:.6f}, {extrinsic_R[1]:.6f}, {extrinsic_R[2]:.6f}, " \
                f"{extrinsic_R[3]:.6f}, {extrinsic_R[4]:.6f}, {extrinsic_R[5]:.6f}, " \
                f"{extrinsic_R[6]:.6f}, {extrinsic_R[7]:.6f}, {extrinsic_R[8]:.6f} ]"
        content = re.sub(
            r'extrinsic_R:\s*\[.*?\]',
            f'extrinsic_R: {R_str}',
            content,
            flags=re.DOTALL
        )
        
        # 设置extrinsic_est_en为false以禁用在线外参估计
        if 'extrinsic_est_en:' in content:
            content = re.sub(
                r'extrinsic_est_en:\s*\w+',
                'extrinsic_est_en: false',
                content
            )
        else:
            # 如果没有这个参数，在extrinsic_T下面添加
            content = re.sub(
                r'(extrinsic_T:.*?\n)',
                r'\1extrinsic_est_en: false\n',
                content
            )
        
        with open(yaml_file, 'w') as f:
            f.write(content)
        
        print(f"[✓] 配置文件已更新")
        return True
    
    except Exception as e:
        print(f"[✗] 更新配置文件失败: {e}")
        return False


def show_updated_config(yaml_file):
    """显示更新后的配置片段"""
    try:
        with open(yaml_file, 'r') as f:
            lines = f.readlines()
        
        print(f"\n[*] 更新后的配置片段:")
        in_extrinsic = False
        for i, line in enumerate(lines):
            if 'extrinsic' in line.lower() or in_extrinsic:
                print(f"    {line.rstrip()}")
                if 'extrinsic_est_en' in line:
                    in_extrinsic = False
                else:
                    in_extrinsic = True
                if 'extrinsic_est_en' in line:
                    break
    except Exception as e:
        print(f"[✗] 读取配置文件失败: {e}")


def main():
    print("=" * 70)
    print("FAST-LIO TF to Extrinsic 自动转换工具")
    print("=" * 70)
    
    # 初始化ROS节点
    try:
        rospy.init_node('tf_to_extrinsic_converter', anonymous=True)
    except rospy.exceptions.ROSException as e:
        print(f"\n[✗] 无法初始化ROS节点: {e}")
        print("    请确保：")
        print("    1. roscore 已运行")
        print("    2. FAST-LIO 已启动")
        print("    3. 运行此脚本时使用: source /opt/ros/noetic/setup.bash && python3 <script>")
        sys.exit(1)
    
    # 获取TF变换
    trans, quat = get_tf_transform('body', 'livox_frame')
    
    if trans is None or quat is None:
        print("\n[✗] 无法获取TF变换，流程中止")
        sys.exit(1)
    
    # 转换为RPY
    euler = quaternion_to_rpy(quat)
    roll_rad, pitch_rad, yaw_rad = euler
    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    yaw_deg = math.degrees(yaw_rad)
    
    print(f"\n[*] 转换后的欧拉角 (RPY):")
    print(f"    Roll:  {roll_deg:8.4f}° ({roll_rad:8.6f} rad)")
    print(f"    Pitch: {pitch_deg:8.4f}° ({pitch_rad:8.6f} rad)")
    print(f"    Yaw:   {yaw_deg:8.4f}° ({yaw_rad:8.6f} rad)")
    
    # 计算旋转矩阵
    rotation_matrix = rpy_to_rotation_matrix(roll_rad, pitch_rad, yaw_rad)
    extrinsic_R_flat = flatten_matrix(rotation_matrix)
    
    print(f"\n[*] 计算的旋转矩阵 (3x3):")
    for i in range(3):
        print(f"    [{rotation_matrix[i][0]:8.6f}, {rotation_matrix[i][1]:8.6f}, {rotation_matrix[i][2]:8.6f}]")
    
    print(f"\n[*] FAST-LIO extrinsic_T 参数:")
    print(f"    extrinsic_T: [ {trans[0]:.6f}, {trans[1]:.6f}, {trans[2]:.6f} ]")
    
    print(f"\n[*] FAST-LIO extrinsic_R 参数 (9元素):")
    print(f"    extrinsic_R: [ {extrinsic_R_flat[0]:.6f}, {extrinsic_R_flat[1]:.6f}, {extrinsic_R_flat[2]:.6f},")
    print(f"                   {extrinsic_R_flat[3]:.6f}, {extrinsic_R_flat[4]:.6f}, {extrinsic_R_flat[5]:.6f},")
    print(f"                   {extrinsic_R_flat[6]:.6f}, {extrinsic_R_flat[7]:.6f}, {extrinsic_R_flat[8]:.6f} ]")
    
    # 更新YAML配置文件
    yaml_file = os.path.join(
        os.path.dirname(__file__),
        '../src/FAST_LIO/config/mid360.yaml'
    )
    yaml_file = os.path.abspath(yaml_file)
    
    success = update_yaml_config(trans, extrinsic_R_flat, yaml_file)
    
    if success:
        show_updated_config(yaml_file)
        print(f"\n[✓] 配置文件更新成功！")
        print(f"\n[*] 后续步骤:")
        print(f"    1. 重新编译FAST-LIO:")
        print(f"       cd /home/nvidia/task5/fastlio_ws && catkin_make")
        print(f"    2. 启动FAST-LIO:")
        print(f"       bash run_task5_fastlio_real.sh")
        print(f"    3. 在RViz中检查点云是否仍然倾斜")
        print(f"\n" + "=" * 70)
        sys.exit(0)
    else:
        print(f"\n[✗] 配置文件更新失败")
        print(f"\n[*] 手动更新 {yaml_file}:")
        print(f"    extrinsic_T: [ {trans[0]:.6f}, {trans[1]:.6f}, {trans[2]:.6f} ]")
        print(f"    extrinsic_R: [ {extrinsic_R_flat[0]:.6f}, {extrinsic_R_flat[1]:.6f}, {extrinsic_R_flat[2]:.6f},")
        print(f"                   {extrinsic_R_flat[3]:.6f}, {extrinsic_R_flat[4]:.6f}, {extrinsic_R_flat[5]:.6f},")
        print(f"                   {extrinsic_R_flat[6]:.6f}, {extrinsic_R_flat[7]:.6f}, {extrinsic_R_flat[8]:.6f} ]")
        print(f"    extrinsic_est_en: false")
        sys.exit(1)


if __name__ == '__main__':
    main()
