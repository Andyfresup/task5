#!/bin/bash

# 自动化执行脚本：启动FAST-LIO并转换TF参数
# 用法：bash auto_workflow.sh

set -e

echo "========================================================================"
echo "FAST-LIO TF参数自动转换工作流"
echo "========================================================================"
echo ""

FASTLIO_WS="/home/nvidia/task5/fastlio_ws"
cd "$FASTLIO_WS"

# 第一步：启动ROS核心（如果还未启动）
echo "[*] 检查ROS状态..."
if ! pgrep -x "roscore" > /dev/null; then
    echo "[*] 启动roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
    echo "[✓] roscore已启动 (PID: $ROSCORE_PID)"
else
    echo "[✓] roscore已在运行"
fi

# 第二步：启动FAST-LIO（后台运行）
echo ""
echo "[*] 启动FAST-LIO系统..."
bash run_task5_fastlio_real.sh > /tmp/fastlio.log 2>&1 &
FASTLIO_PID=$!
echo "[✓] FAST-LIO已启动 (PID: $FASTLIO_PID)"
echo "    日志: /tmp/fastlio.log"

# 等待TF树建立
echo ""
echo "[*] 等待TF树建立 (10秒)..."
sleep 10

# 第三步：激活ROS环境并运行转换脚本
echo ""
echo "[*] 运行TF转换脚本..."
source devel/setup.bash
python3 tools/auto_convert_tf_to_extrinsic.py

CONVERT_RESULT=$?

# 第四步：根据结果提示后续操作
echo ""
echo "========================================================================"
if [ $CONVERT_RESULT -eq 0 ]; then
    echo "[✓] 转换完成！现在需要重新编译FAST-LIO"
    echo ""
    echo "[*] 停止FAST-LIO..."
    kill $FASTLIO_PID 2>/dev/null || true
    sleep 2
    
    echo ""
    echo "[*] 开始编译FAST-LIO..."
    catkin_make
    
    echo ""
    echo "[✓] 编译完成！"
    echo ""
    echo "[*] 启动更新后的FAST-LIO..."
    bash run_task5_fastlio_real.sh
else
    echo "[✗] 转换失败，请检查上面的错误信息"
    echo ""
    echo "[*] 停止FAST-LIO..."
    kill $FASTLIO_PID 2>/dev/null || true
fi

echo "========================================================================"
