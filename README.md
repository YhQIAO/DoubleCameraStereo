# DoubleCameraStereo
双目立体视觉
双目的SfM与稀疏重建
密集匹配建立稠密点云（待实现）

## 环境要求
- opencv4（我的是4.5.1）
- pcl（我的是pcl1.11.1）
- 我的代码编译环境为visual studio 2019

## 代码说明
1. Camera相机类
2. SIFTMatcher SIFT特征匹配类
3. SfMSolver sfm解算类
4. SparseReconstructor 稀疏重建类

## 测试结果
特征初匹配
![image](https://github.com/YhQIAO/blog_images/blob/main/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202021-04-27%20160826.png)
随机采样一致性，根据基础矩阵约束后的特征匹配
位姿恢复与稀疏点云重建
