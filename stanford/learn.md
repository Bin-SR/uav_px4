# Standford-uav GRaD-Nav

## 1.预备知识  

### 1.1 渲染 
**NeRF 和 3DGS**  
#### NeRF   
全称是 Neural Radiance Fields，中文叫神经辐射场。  
它的核心思想是：

利用神经网络学习三维空间中的位置与观察方向到颜色和体密度的映射。  
$$F_\theta(\mathbf{x},\mathbf{d}) \rightarrow (\mathbf{c},\sigma)$$
其中：
- $(\mathbf{x}=(x,y,z))$：三维空间位置；
- $(\mathbf{d})$：观察方向；
- $(\mathbf{c}=(r,g,b))$：颜色；
- $(\sigma)$：体密度；
- $(\theta)$：神经网络参数。
训练完成后，可以通过体渲染（Volume Rendering）得到某个相机视角下的图像。  

NeRF 的意义在于：不需要传统意义上显式构建完整的三角网格，也可以学习一个能够进行新视角渲染的三维场景表示。  

#### 3D Gaussian Splatting（3DGS）  
与 NeRF 都可以用于三维场景表示，但实现方式不同。  
NeRF 主要通过神经网络隐式表示场景  
3DGS 则通过大量具有空间位置、尺度、旋转、透明度和颜色属性的三维高斯分布来显式表示场景  
单个高斯可以表示为：
$G(\mathbf{x})= \exp\left(-\frac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^T\Sigma^{-1}(\mathbf{x}-\boldsymbol{\mu})\right)$  

大量高斯组合起来，就能表示复杂的三维场景  

Nerfstudio仓库，它主要承担三维场景表示与相关计算的基础设施角色  
gsplat：提供高斯泼溅相关的高效计算能力  
GRaD-Nav：在这些基础上构建无人机视觉导航和强化学习系统    

### 1.2 GRaD-Nav  
#### 要解决的问题：  
传统无人机通常的步骤：感知周围环境，获取障碍物与目标位置，路径规划，生成轨迹，控制无人机跟踪轨迹  
GRad-Nav: 希望无人机可以直接根据**视觉信息**学习导航策略，而不是完全以来传统的显式地图、路径规划及轨迹控制流  
#### 用到的技术  
3D Gaussian Splatting(3DGS), Differentiable Dynamics(可微分动力学), RL强化学习  
#### 核心思想  
技术路线可以理解成：3DGS场景->视觉观测->导航策略->无人机控制  
关键设计：**把可微分的场景渲染和无人机动力学，纳入强化学习的优化过程**  
传统的强化学习训练中，策略网络输出actions后，无人机在仿真环境中运动，获得下一时刻的obs，但是环境中的渲染和动力学通常被当作不可微分的黑盒  
GRaD-Nav则利用可微分模块，让**梯度**能够通过视觉观测和动力学过程传播，为策略优化提高更丰富的梯度信息  
#### 系统架构        
- 环境表示-3DGS  
- 无人机动力学-Differentiable Dynamics  
一般来说，四旋翼的状态可以写成: $s_t = [p_t, v_t, q_t, \omega_t]$  
其中，$p_t$表示位置，$v_t$表示线速度，$q_t$表示姿态四元数，$\omega_t$表示角速度  
动作$a_t$可以是期望推力和例句，也可以是其他控制量，具体取决于代码中的动力学接口  
动力学表达式：$s_{t+1}=f(s_t,a_t)$，并且还要求可微，即
$$
\frac{\partial s_{t+1}}{\partial s_t}
,\qquad
\frac{\partial s_{t+1}}{\partial a_t}
$$

- 视觉策略网络-Actor-Critic
- CENet-Context aided Estimator Network  
CENet 的作用是帮助策略估计环境变化及相关上下文信息,目的就是帮助策略更好地适应环境变化。
#### 关于坐标系的问题
Nerfstudio 导出的场景存在坐标归一化问题，不能直接把原始坐标当作无人机导航坐标。  
在CUSTOM_SCENE.md中，在GPT的第一次问答中的3.4
