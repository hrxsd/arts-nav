#! /usr/bin/env python
from rda_core_1 import rda_core_1
import rospy

# --- 主程序入口 ---
if __name__ == '__main__':
    try:
        rda = rda_core_1()
        # 检查 rda 对象是否成功初始化 (例如，action server 连接成功)
        if hasattr(rda, 'move_base_client'):
             rda.control()
        else:
             rospy.logerr("rda_core initialization failed, shutting down.")
    except rospy.ROSInterruptException:
        pass