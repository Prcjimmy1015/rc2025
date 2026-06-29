# VMware Ubuntu 22.04 双网卡配置指南

## 目标

| 功能 | 说明 |
|------|------|
| 上网 | 虚拟机通过宿主机无线网卡访问互联网 |
| 访问有线设备 | 虚拟机以 IP `192.168.123.222` 访问 `192.168.123.161`、`192.168.123.100` 等有线网段设备 |
| SSH 连接 | 宿主机可以正常通过 SSH 连接虚拟机 |

## 前提条件

- VMware Workstation 已安装
- 宿主机有无线网卡和有线网卡，且都能正常工作
- Ubuntu 22.04 虚拟机已创建并可以启动

---

## 第一步：修改 VMware NAT 子网

> **为什么要做这步？** VMware 默认的 NAT 子网是 `192.168.123.0/24`，和有线网段相同，会导致两个网卡在同一个子网，Linux 无法正确区分流量该走哪张网卡。必须把 NAT 改到不同的子网。

### 1.1 打开虚拟网络编辑器

1. 在 VMware 主界面，点击顶部菜单栏的 **编辑**
2. 点击 **虚拟网络编辑器**
3. 点击右下角的 **更改设置** 按钮
   - 如果弹出 UAC 提示，点 **是** 允许管理员权限

### 1.2 修改 VMnet8 的子网

1. 在列表中选中 **VMnet8**（类型显示为 NAT 模式）
2. 在下方 **子网 IP** 栏中，将内容改为：`192.168.200.0`
3. 确认 **子网掩码** 为：`255.255.255.0`
4. 点击 **NAT 设置** 按钮，确认网关 IP 为 `192.168.200.2`
   - 如果不是，手动改为 `192.168.200.2`
5. 点击 **确定** 关闭 NAT 设置
6. 点击 **确定** 保存虚拟网络编辑器的修改

---

## 第二步：配置虚拟机的网络适配器

> **为什么要做这步？** 需要两张虚拟网卡分别对应两个用途：NAT 上网 + 桥接访问有线设备。

### 2.1 关闭虚拟机

1. 在虚拟机内点击右上角，选择 **关机**
   - 或者在 VMware 主界面，右键虚拟机 → **电源** → **关机**

### 2.2 打开虚拟机设置

1. 在 VMware 主界面，选中你的虚拟机
2. 点击 **编辑虚拟机设置**

### 2.3 配置第一个网络适配器（NAT）

1. 在硬件列表中找到 **网络适配器**（通常叫"网络适配器"）
2. 选中它，在右侧选择 **NAT 模式（N）：用于共享主机的 IP 地址**
3. 确保勾选了 **启动时连接**

### 2.4 添加第二个网络适配器（桥接）

1. 点击窗口下方的 **添加** 按钮
2. 在弹出窗口中选择 **网络适配器**，点击 **完成**
3. 选中刚添加的 **网络适配器 2**
4. 在右侧选择 **桥接模式（B）：直接连接物理网络**
5. 确保勾选了 **启动时连接**

### 2.5 确认桥接到正确的物理网卡

1. 回到 VMware 主界面，点击 **编辑** → **虚拟网络编辑器**
2. 选中 **VMnet0**（桥接模式）
3. 点击 **桥接到** 下拉框，选择宿主机的 **有线网卡**
   - 一般名称中带有 "Ethernet"、"以太网"、"Realtek"、"Intel" 等字样
   - 不要选无线网卡（名称中通常有 "Wi-Fi"、"Wireless"、"WLAN" 等字样）
4. 点击 **确定** 保存

---

## 第三步：启动虚拟机并查看网卡名称

### 3.1 启动虚拟机

在 VMware 中点击 **开启此虚拟机**

### 3.2 打开终端

在虚拟机中按 `Ctrl + Alt + T` 打开终端

### 3.3 查看网卡名称

输入以下命令：

```bash
ip link show
```

你会看到类似这样的输出：

```
1: lo: <LOOPBACK,UP,LOWER_UP> ...
2: ens33: <BROADCAST,MULTICAST,UP,LOWER_UP> ...    ← 这是第一个网卡（NAT）
3: ens37: <BROADCAST,MULTICAST,UP,LOWER_UP> ...    ← 这是第二个网卡（桥接）
```

> **注意**：你的网卡名称可能不同，比如 `ens33`/`ens38` 或 `ens33`/`ens37`。记住这两个名字，后面要用。

### 3.4 确认哪个网卡是 NAT，哪个是桥接

输入以下命令：

```bash
ip addr show
```

查看输出：
- IP 地址为 `192.168.200.xxx` 的那个是 **NAT 网卡**（ens33）
- IP 地址为 `192.168.123.xxx` 的那个是 **桥接网卡**（ens37）

> 如果两个网卡的 IP 都在 `192.168.123.xxx`，说明第一步的 NAT 子网没有改成功，请回到第一步重新操作。

---

## 第四步：配置 netplan 网络文件

### 4.1 查看现有配置文件名

```bash
ls /etc/netplan/
```

你会看到一个文件，比如 `01-network-manager-all.yaml`。记住这个名字。

### 4.2 编辑配置文件

```bash
sudo nano /etc/netplan/01-network-manager-all.yaml
```

> 如果你的文件名不同，把上面的文件名替换成你实际的文件名。

### 4.3 清空原有内容，写入以下配置

> **重要**：YAML 文件对缩进非常敏感！每一层缩进必须是 **2个空格**，不能用 Tab 键！

> **为什么要加 `routes`？** `192.168.123.0/24` 的子网路由已经覆盖了 `192.168.123.100`，但添加一条 `/32` 精确主机路由可以确保即使存在路由冲突，到 `192.168.123.100` 的流量也**强制**走 `ens37`（桥接网卡），不会被其他路由规则干扰。

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    ens33:
      dhcp4: true
    ens37:
      dhcp4: false
      addresses:
        - 192.168.123.222/24
      routes:
        - to: 192.168.123.100/32
          via: 192.168.123.100
```

> **替换说明**：
> - 如果你的 NAT 网卡不叫 `ens33`，把上面的 `ens33` 改成你实际的网卡名
> - 如果你的桥接网卡不叫 `ens37`，把上面的 `ens37` 改成你实际的网卡名
> - 如果你需要访问的 IP 不是 `192.168.123.222`，修改 `addresses` 中的 IP
> - 如果需要强制走 ens37 的目标 IP 不是 `192.168.123.100`，修改 `routes` 中的 `to` 和 `via` 字段

### 4.4 保存并退出

1. 按 `Ctrl + O` 保存
2. 按 `Enter` 确认文件名
3. 按 `Ctrl + X` 退出编辑器

### 4.5 应用配置

```bash
sudo netplan apply
```

- 如果没有任何输出，说明配置成功
- 如果有报错，说明 YAML 格式有问题（通常是缩进错误），请回到 4.3 重新检查

---

## 第五步：验证网络连通性

### 5.1 验证上网（通过 NAT / 无线网卡）

```bash
ping -c 3 8.8.8.8
```

**期望结果**：能看到 `64 bytes from 8.8.8.8` 的回复

**如果 ping 不通**：
1. 检查 VMware 的 NAT 服务是否启动（Windows 宿主机上按 `Win + R`，输入 `services.msc`，找到 `VMware NAT Service`，确认状态为"正在运行"）
2. 检查路由：`ip route show`，确认有 `default via 192.168.200.2 dev ens33`

### 5.2 验证访问有线设备

```bash
ping -c 3 192.168.123.161
ping -c 3 192.168.123.100
```

**期望结果**：都能看到 `64 bytes from 192.168.123.xxx` 的回复

#### 5.2.1 验证 192.168.123.100 强制走 ens37

使用 `ip route get` 查看内核实际使用的路由（无需安装额外工具）：

```bash
ip route get 192.168.123.100
```

**期望结果**：

```
192.168.123.100 dev ens37 src 192.168.123.222 uid ...
```

> 输出中必须出现 `dev ens37`，说明到 `192.168.123.100` 的流量确实走了桥接网卡。如果显示 `dev ens33`，说明 ensp 的精确路由未生效，请回到 4.3 检查 `routes` 配置是否正确写入。

#### 5.2.2 如果 ping 不通

1. 确认桥接网卡 IP 是否正确：`ip addr show ens37`
2. 确认 VMware 桥接模式指定的是有线网卡（回到 2.5 检查）
3. 确认目标设备（如 192.168.123.161、192.168.123.100）在有线网络中是可达的

### 5.3 验证路由表

```bash
ip route show
```

**期望结果**：

```
default via 192.168.200.2 dev ens33          ← 默认路由走 NAT（上网）
192.168.123.100 dev ens37 scope link          ← 强制路由（精确指定 192.168.123.100 走 ens37）
192.168.123.0/24 dev ens37 ...               ← 有线网段走桥接
192.168.200.0/24 dev ens33 ...               ← NAT 网段
```

> **关键检查**：
> - 默认路由（default）必须走 ens33（NAT 网卡），否则上不了网
> - `192.168.123.100 dev ens37 scope link` 这条精确路由必须存在，确保到 `192.168.123.100` 的流量强制走桥接网卡

---

## 第六步：配置 SSH 连接

### 6.1 安装 SSH 服务（如果还没装）

```bash
sudo apt update
sudo apt install openssh-server -y
```

### 6.2 检查 SSH 服务状态

```bash
sudo systemctl status ssh
```

**期望结果**：看到 `Active: active (running)` 字样

### 6.3 从宿主机连接虚拟机

在 **宿主机** 上打开终端（Windows 用 PowerShell 或 CMD），输入：

```bash
ssh 你的虚拟机用户名@192.168.200.128
```

> 将 `你的虚拟机用户名` 替换为虚拟机实际的用户名，IP 替换为 ens33 的实际 IP（用 `ip addr show ens33` 查看）

**期望结果**：成功登录虚拟机

### 6.4 从有线网段 SSH 连接（可选）

如果需要从 `192.168.123.x` 网段的设备 SSH 连接虚拟机：

```bash
ssh 你的虚拟机用户名@192.168.123.222
```

---

## 第七步：重启验证

最后，重启虚拟机确认所有配置在重启后依然生效：

```bash
sudo reboot
```

重启后再次执行：

```bash
ping -c 2 8.8.8.8
ping -c 2 192.168.123.161
ping -c 2 192.168.123.100
ip route show
```

确认结果都和之前一致即可。

---

## 常见问题排查

### 问题1：能上网但 ping 不通有线网段设备（如 192.168.123.161、192.168.123.100）

**原因**：桥接模式没有指定到宿主机的有线网卡

**解决**：回到第二步的 2.5，确认 VMnet0 桥接到的是有线网卡而非无线网卡

### 问题2：能 ping 通有线网段设备但上不了网

**原因**：桥接网卡抢了默认路由

**解决**：

```bash
# 查看路由
ip route show

# 如果有 "default via ... dev ens37" 这样的行，手动删除
sudo ip route del default dev ens37

# 确认默认路由走 ens33
ip route show
```

然后检查 netplan 配置，确认 ens37 没有设置 `gateway4`

### 问题3：netplan apply 报错

**原因**：YAML 格式错误（缩进不对、用了 Tab 键等）

**解决**：

1. 重新打开配置文件：`sudo nano /etc/netplan/01-network-manager-all.yaml`
2. 确认所有缩进都是 **2个空格**，没有 Tab 键
3. 确认冒号后面有空格（如 `dhcp4: true`，不是 `dhcp4:true`）
4. 保存后重新 `sudo netplan apply`

### 问题4：宿主机 SSH 连不上虚拟机

**解决**：

1. 确认虚拟机内 SSH 服务运行中：`sudo systemctl status ssh`
2. 确认虚拟机防火墙放行 22 端口：`sudo ufw allow 22`
3. 确认宿主机能 ping 通虚拟机 IP：`ping 192.168.200.128`
4. 如果 ping 不通，检查 VMware 的 NAT 服务是否在运行

### 问题5：重启后桥接网卡 IP 丢失

**原因**：netplan 配置没有正确保存或应用

**解决**：

```bash
# 查看当前配置
sudo cat /etc/netplan/01-network-manager-all.yaml

# 确认 ens37 的 addresses 配置还在
# 如果不在，重新按第四步配置
```

### 问题6：192.168.123.100 走了错误的网卡（如 ens33 而不是 ens37）

**原因**：精确路由（`/32`）未配置或未生效，导致流量走了默认路由

**解决**：

```bash
# 1. 查看当前到 192.168.123.100 的路由决策
ip route get 192.168.123.100

# 2. 如果输出显示 dev ens33，说明精确路由缺失，检查 netplan 配置
sudo cat /etc/netplan/01-network-manager-all.yaml

# 3. 确认 ens37 下有以下 routes 块：
#      routes:
#        - to: 192.168.123.100/32
#          via: 192.168.123.100

# 4. 如果缺失，重新编辑配置文件并应用
sudo nano /etc/netplan/01-network-manager-all.yaml
sudo netplan apply

# 5. 再次验证
ip route get 192.168.123.100
```

### 问题7：必须关闭 ens33 才能操作 D1 机械臂（DDS 通信失败）

**现象**：可以 ping 通 `192.168.123.100`，但机械臂程序（DDS/CycloneDDS）无法通信，`d1_get_arm_joint_angle` 返回空或报错，必须 `sudo ip link set ens33 down` 后才能正常操作。

**原因**：机械臂使用 Cyclone DDS 进行通信（组播地址 `239.255.0.1:7401`），DDS 库在内部会自己枚举所有网卡并选择一个"最佳"接口，**不会遵循系统路由表**。当 ens33 存在时，DDS 可能绑定到 ens33（NAT 网卡），导致组播发现包无法到达有线网络中的机械臂。即使系统路由表已经正确配置 `224.0.0.0/4` 指向 ens37，DDS 仍然会走它自己选择的网卡。

**解决**：

#### 步骤 A：确认 DDS 组播地址

在关机 ens33 的状态下运行机械臂程序，用 tcpdump 抓包确认组播地址：

```bash
sudo tcpdump -i ens37 -n 'multicast' -c 20
```

期望看到 `192.168.123.100 > 239.255.0.1.7401` 的 UDP 包。

#### 步骤 B：添加组播路由（系统层面）

在 netplan 中为 ens37 添加 `224.0.0.0/4` 路由，覆盖所有 IPv4 组播地址：

```yaml
ens37:
  routes:
    - to: 192.168.123.100/32
      via: 192.168.123.100
    - to: 224.0.0.0/4          # ← 组播路由
      via: 192.168.123.100
```

应用配置:
```bash
sudo netplan apply
ip route get 239.255.0.1    # 确认输出包含 dev ens37
```

#### 步骤 C：调整反向路径过滤

```bash
sudo sysctl -w net.ipv4.conf.ens37.rp_filter=0
sudo sysctl -w net.ipv4.conf.ens33.rp_filter=2
```

持久化到 `/etc/sysctl.conf`：
```
net.ipv4.conf.ens37.rp_filter=0
net.ipv4.conf.ens33.rp_filter=2
```

#### 步骤 D：强制 DDS 绑定 ens37（关键）

方法1：通过环境变量（推荐，在 Python 脚本中设置）

```python
import os

_CYCLONEDDS_ENV = {
    'CYCLONEDDS_URI': '<CycloneDDS><Domain><General><Interfaces>'
                      '<NetworkInterface name="ens37"/>'
                      '</Interfaces></General></Domain></CycloneDDS>'
}

# 在 subprocess.run 中注入
env = os.environ.copy()
env.update(_CYCLONEDDS_ENV)
subprocess.run(cmd, env=env, ...)
```

方法2：通过 XML 配置文件

创建 `$HOME/.config/cyclonedds/config.xml`：
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:schemaLocation="https://cdds.io/config
    https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="0">
    <General>
      <Interfaces>
        <NetworkInterface name="ens37" priority="default" multicast="true" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
```

> **注意**：`$HOME/.config/cyclonedds/config.xml` 可能不被 `sudo` 运行时读取（因为 HOME 环境变量变了）。推荐在 Python 脚本中通过 `CYCLONEDDS_URI` 环境变量方式注入，更加可靠。

#### 步骤 E：验证

```bash
# 不关 ens33 下运行机械臂程序
cd /path/to/d1_arm/build
sudo python3 d1_arm.py

# 期望能看到关节角度输出（如 "-89.6,0.5,40.5,..."），而不是空值
```

### 问题8：/etc/cyclonedds.xml 配置不生效

**原因**：Cyclone DDS 默认不读取 `/etc/cyclonedds.xml`。默认搜索路径为：
1. `$CYCLONEDDS_URI` 环境变量（最高优先级）
2. `$HOME/.config/cyclonedds/config.xml`
3. `/etc/cyclonedds.xml`（某些版本可能不支持）

且 `sudo` 运行时会改变 `$HOME`，导致 `~/.config/cyclonedds/config.xml` 也读取失败。

**解决**：使用 `CYCLONEDDS_URI` 环境变量方式（见问题7的步骤 D 方法1），这是最可靠的方式。

---

## 配置文件速查

### VMware 设置

| 项目 | 设置 |
|------|------|
| VMnet8 (NAT) 子网 | 192.168.200.0/255.255.255.0 |
| VMnet8 网关 | 192.168.200.2 |
| VMnet0 (桥接) 桥接到 | 宿主机有线网卡 |
| 虚拟机网络适配器 1 | NAT 模式 |
| 虚拟机网络适配器 2 | 桥接模式 |

### 虚拟机网络

| 网卡 | VMware 模式 | IP | 用途 |
|------|------------|-----|------|
| ens33 | NAT | 192.168.200.128 (DHCP) | 上网 + 宿主机 SSH |
| ens37 | 桥接 | 192.168.123.222 (静态) | 访问有线网段设备（如 192.168.123.161、192.168.123.100） |

### Netplan 配置文件 `/etc/netplan/01-network-manager-all.yaml`

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    ens33:
      dhcp4: true
    ens37:
      dhcp4: false
      addresses:
        - 192.168.123.222/24
      routes:
        - to: 192.168.123.100/32
          via: 192.168.123.100
        - to: 224.0.0.0/4           # 组播路由（DDS 通信需要）
          via: 192.168.123.100
```

### DDS 组播通信（重要）

如果使用基于 DDS（CycloneDDS）的设备（如 Unitree D1 机械臂），需要在所有 C++ 源码中将：
```cpp
ChannelFactory::Instance()->Init(0);
```
改为指定网卡接口：
```cpp
ChannelFactory::Instance()->Init(0, "ens37");
```
然后重新编译。这比使用环境变量或 XML 配置文件更可靠，覆盖所有调用方式。

### 其他配置

`/etc/sysctl.conf` 追加（防止组播包被内核丢弃）：
```
net.ipv4.conf.ens37.rp_filter=0
net.ipv4.conf.ens33.rp_filter=2
```

