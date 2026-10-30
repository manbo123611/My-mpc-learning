import mujoco        # 导入物理引擎核心库，用于处理物理计算与动力学仿真
import mujoco.viewer # 导入内置可视化渲染模块，用于显示3D场景
import time          # 导入时间模块，用于控制主循环频率

# 从XML字符串解析并构建系统物理模型(MjModel)
# 该对象存储系统的静态配置（如质量、几何尺寸、拓扑结构），在仿真中保持不变
model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody> <!-- 定义世界主体层级结构 -->
    <light pos="0 0 3" dir="0 0 -1"/> <!-- 定义光源位置与照射方向 -->
    <geom type="plane" size="5 5 0.1"/> <!-- 定义地面平面及其碰撞属性 -->
    <body pos="0 0 2"> <!-- 定义一个刚体，初始质心位置为(0,0,2) -->
      <joint name="ball_joint" type="free"/> <!-- 赋予刚体6个自由度(3个平移、3个旋转)，允许自由落体 -->
      <geom type="sphere" size="0.2" rgba="0 1 0 1" mass="20"/> <!-- 定义球体几何属性:半径0.2'RGBA颜色(绿色)'质量20kg -->
    </body>
  </worldbody>
  <actuator>
  <motor joint="ball_joint" gear=" 0 0 1 0 0 0"/> <!--gear 的六个参数依次代表(X、Y、Z 轴平移力）和（绕 X、Y、Z 轴的旋转力矩）-->
  </actuator>
</mujoco>
""")
# 初始化系统动态数据(MjData)
# 该对象存储仿真过程中的所有实时状态变量（如位置、速度、加速度及控制输入）
data = mujoco.MjData(model)

# 启动被动可视化窗口，创建上下文管理器
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 仿真主循环：只要渲染窗口处于开启状态，则持续运行
    
    while viewer.is_running():
        # 关键改动：把控制力放进循环里，每一帧都重新赋值
        data.ctrl[0] = 196.2
        # 执行单步物理积分计算
        # 引擎根据当前状态与受力计算下一步的动力学结果，并更新到 data 对象中
        mujoco.mj_step(model, data)
        
        # 状态同步：将 data 中最新的物理状态同步至渲染引擎，刷新画面
        viewer.sync()
        
        # 线程休眠0.01秒（10毫秒）
        # 限制循环执行速度，使仿真运行频率与真实时间流逝大致匹配，没有的话意味着小球会迅速落地，肉眼无法观察到下落过程
        time.sleep(0.01)
        
        # 读取并打印 data 对象中 qpos（位置状态）数组的索引2元素，即刚体的Z轴高度
        print("当前高度：", data.qpos[2])