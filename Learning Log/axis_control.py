import mujoco        # 取用mujoco这个库，类似numpy，有这个才能执行mujoco代码
import mujoco.viewer # 拉入一个窗口，用来观察仿真主体的运动情况
import time          # 引入时间概念，模拟现实时间

# 以下为XML字符串用于构建仿真的物理模型(MjModel)
# 加载模型：类似于做图纸，MjModel本质是提供一个图纸模板，from_xml_string是读入xml字符串内设置的参数并按照模板写成图纸，最后挂上model这个图纸号，后续所有物理参数都从这里读，不能随便改
model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody> <!--创建一个世界,在这个世界中规定各参数-->
    <light pos="0 0 3" dir="0 0 -1"/> <!--创建一个光源,规定他的空间所处位置以及光照方向-->
    <geom type="plane" size="5 5 0.1"/> <!--创建一个地面,规定他的长宽高-->
    <body pos="0 0 2"> <!--仿真主体初始位置,对应空间坐标-->
      <joint name="ball_joint" type="free"/> <!--给仿真主体一个名字,方便让力知道施加对象是谁,然后规定它可以进行任何运动-->
      <geom type="sphere" size="0.2" rgba="0 1 0 1" mass="20"/> <!--把仿真主体定义为小球,然后规定他的尺寸(半径为0.2),接着是颜色和质量(单位:kg)-->
    </body> <!--仿真主体构建到这里结束-->
  </worldbody> <!--世界构建到这里结束-->
  <actuator> <!--引入力的概念-->
  <motor joint="ball_joint" gear=" 1 0 0 0 0 0"/> <!--明确力作用对象是谁,朝那个方向或者绕哪个轴旋转,大小多少(这里是系数，类似矢量数乘)-->
  <motor joint="ball_joint" gear=" 0 1 0 0 0 0"/>
  <motor joint="ball_joint" gear=" 0 0 1 0 0 0"/>
  </actuator> <!--有关力的参数设置到这里结束-->
</mujoco>
""")

last_print_time = time.time()
def ball_gravity(m):
    mg = m*(-model.opt.gravity[2]) #计算重力大小，m是小球质量，model.opt.gravity[2]是重力加速度大小
    return mg #利用定义一个函数来避免魔法数字，改变质量的同时就可以通过函数计算出重力，不用手动重算。

data = mujoco.MjData(model) # 该操作的本质是MjData是创建仓库（从内存中分出空间存储物理量参数），然后给这个仓库挂上data这个门牌号

with mujoco.viewer.launch_passive(model, data) as viewer: # 拉起仿真窗口，观察运动
    #构建循环体，不断更新和写入数据
    
    t0 = time.time() # 记录仿真开始时间
    while viewer.is_running():

        forces = [50.0, 50.0, ball_gravity(model.body_mass[1])] # 列表存储三个力的大小，前两个是恒定的50N，第三个是小球重力大小

        for i in range(3): # 通过for循环给三个力赋值
            data.ctrl[i] = forces[i] # 通过data.ctrl数组拉取列表中的值给三个力赋值，注意这里是直接赋值，不是累加

        mujoco.mj_step(model, data) #获取实时参数和运动时间，通过公式计算加速度，积分出实时的速度和位移，再次存回data，覆写旧数据，本质是函数，底层涉及指针
         
        viewer.sync() #将data里物理量表现出来的物理现象实时绘画出来
         
        time.sleep(0.0016) #休眠 1.6 ms，用来补足一帧（注释原来写 0.01 是错的）
        
        if time.time() - last_print_time > 0.01: # 每隔0.01秒打印一次实时高度和速度，降频打印，但要避免错过打印窗口
            last_print_time = time.time()
            # 打印实时高度，data.qpos数组中包含7个状态量，对应第24行代码，而数组索引从0(X轴坐标)开始，2就对应小球的Z轴坐标
            print("当前高度:",data.qpos[2])
            
            print("X轴速度:",data.qvel[0]) #打印小球X方向的速度大小
            print("Y轴速度:",data.qvel[1]) #打印小球Y方向的速度大小
            print("Z轴速度:",data.qvel[2]) #打印小球Z方向的速度大小

            print("X角速度:",data.qvel[3]) #打印小球X方向的角速度大小
            print("Y角速度:",data.qvel[4]) #打印小球Y方向的角速度大小
            print("Z角速度:",data.qvel[5]) #打印小球Z方向的角速度大小

            if time.time() - t0 > 1: #仿真时间超过1秒就退出循环
                break


   

