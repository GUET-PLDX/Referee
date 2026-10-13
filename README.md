# Referee

底盘侧 RoboMaster 2026 裁判系统串口解析器。模块解析官方 V2.0.0 命令
`0x0001`、`0x0003`、`0x0101`、`0x0201`、`0x0202`、`0x0203`、
`0x0204`、`0x0206`、`0x0208`、`0x0209`，并把最近一次有效数据发布为
`RobotGameRefereePack`。

摘要携带 `source_command_id`、`source_valid_mask` 和 `referee_online`。
只有完整且长度正确的官方数据包会更新对应源位；裁判串口离线时清空有效位并发布
离线状态，不会把超时当作新的业务数据。

## Required Hardware

cmd

## Constructor Arguments

None

## Template Arguments

None

## Depends

None

## LibXR 2.0 静态装配

使用 xrobot 1.0.1 的 `module` / 有序 `args`；构造函数显式注入外设与服务，实例按依赖顺序创建。参数名和顺序以头文件中的构造函数为准，固件示例见 `User/RobotConfig/`。

保留 PLDX 控制、协议、超时与离线保护。 `OnMonitor()` 由生成的监控循环直接调用。
