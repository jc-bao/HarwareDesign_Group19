# ReadMe

# 2020硬设报告

## 盲人自动导航系统(头盔部分)

#### 开发说明
* Headgear_Pro向下兼容Headgear_Lite,因此建议直接使用Headgear_Pro进行开发,在Lite版本中只需要一点修改即可
#### 说明展示视频
* https://cloud.tsinghua.edu.cn/f/f6ff1bf1e7b34883bfde/
#### 功能特点:
  * 帮助盲人导航,通过衣架反射实现人的自动转头,方便残疾人凭借直觉直接行走.
  * 帮助盲人避障,通过头两边的震动模块的不同震动模式(间歇震动、长震动).实现障碍物以及停止行走的提示.
  * 提供头部转角获取,帮助导航.
#### 主要工作:
  * 头套硬件部分
    * 装置1(Lite版)
      * 金属件的制作以及装置的固定
      * 鸭舌帽的改装
      * 电路安装
      * 实验可行性验证与硬件的调整
      * 调试实现参数
    * 装置2(Pro版)
      * 头盔整体的建模以及相应的制作
      * 液压装置的设计与制作
      * 电路及相关模块的安装
      * 验证实验、参数的调试
  * 头套软件部分
    * 转向部分
      * 实现方案1的定时转向以及电机力度调节
      * 实现方案2的液压控制以及压力自动适应调节
    * 震动提示部分
      * 实现电机的长震动与间歇震动并封装为相应移动停止函数
    * 接口部分
      * 提供转向、震动接口
      * 提供三维角度获取接口
      * 提供占空比、震动模式修改接口

#### 所用模块
  * 方案1(EYE Lite)
    * Arduino UNO * 1
    * MPU6050 * 1
    * 电机 * 2
    * 减速箱&摩擦轮 * 2
    * 震动马达模块 * 2
    * L298N 电机控制模块 * 2
    * 12864液晶模块(调试用) * 1
    * DX-BT18蓝牙模块(调试用) * 1
    * 9V电池*2
  * 方案2(EYE Pro)
    * Arduino UNO * 1
    * MPU6050 * 1
    * 电机 * 2
    * 减速箱 * 2
    * 针筒 * 4 输液延长管 * 2
    * 震动马达模块 * 2
    * L298N 电机控制模块 * 2
    * 薄膜压力电阻&线性电压转换模块 * 2
    * 12864液晶模块(调试用) * 1
    * DX-BT18蓝牙模块(调试用) * 1
    * USB升压模块
#### 所用技术:
  * 液压传动
  * 压力自动控制技术
#### 参考资料
  * 实验原理部分
    * Matsue R, Sato M, Hashimoto Y, et al. “Hanger reflex”:A reflex motion of a head by temporal pressure for wearable interface[C]. society of instrument and control engineers of japan, 2008: 1463-1467.
    * Sato M, Matsue R, Hashimoto Y, et al. Development of a head rotation interface by using Hanger Reflex[C]. robot and human interactive communication, 2009: 534-538.
    * Kajimoto官方账号: https://www.youtube.com/watch?v=on22yoI40TI
  * MPU6050模块的使用参考https://zhuanlan.zhihu.com/p/20082486
