# 项目介绍
使用输入的模板，匹配点云中存在的物体，匹配成功后进行跟踪

## 结构
- 文件读取
- 模板匹配
- 模板跟踪

---

## 结构细分

### 文件读取

- 读写单个点云文件 (一行代码 算了)
  
- 读取多个点云文件 path ---> vector


### 模板匹配

- 初始化模板与参数

- 去除地面
- RANSAC匹配模板
- ICP对其模板

- tool


### 模板跟踪

- 初始化模板和参数

- 预测

- 更新
