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
