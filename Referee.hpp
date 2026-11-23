/**
 * @file Referee.hpp
 * @author w (2055498415@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: RoboMaster 2027 referee protocol V1.0.0 parser
depends: []
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "CMD.hpp"
#include "crc.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "queue/mpmc_queue.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/**
 * @brief 裁判系统类，用于接收、解包裁判系统
 *
 * @note 位域在Linux gcc/clang和arm-none-eabi-gcc
 *       的行为是一样的
 */
class Referee {
 public:
  static constexpr uint32_t REFEREE_RX_TIMEOUT_MS = 10;
  static constexpr uint32_t MAX_BUY_BULLET_NUM = (1U << 11U) - 1U;
  static constexpr uint32_t MAX_REMOTE_BUY_BULLET_TIMES = (1U << 4U) - 1U;

  /**
   * @brief 用于0x0104
   *
   */
  enum class RobotID : uint8_t {
    REF_BOT_RED_HERO = 1,
    REF_BOT_RED_ENGINEER = 2,
    REF_BOT_RED_INFANTRY_1 = 3,
    REF_BOT_RED_INFANTRY_2 = 4,
    REF_BOT_RED_INFANTRY_3 = 5,
    REF_BOT_RED_DRONE = 6,
    REF_BOT_RED_SENTRY = 7,
    REF_BOT_RED_DART = 8,
    REF_BOT_RED_RADAR = 9,
    REF_BOT_RED_OUTPOST = 11,
    REF_BOT_RED_BASE = 10,
    REF_BOT_BLU_HERO = 101,
    REF_BOT_BLU_ENGINEER = 102,
    REF_BOT_BLU_INFANTRY_1 = 103,
    REF_BOT_BLU_INFANTRY_2 = 104,
    REF_BOT_BLU_INFANTRY_3 = 105,
    REF_BOT_BLU_DRONE = 106,
    REF_BOT_BLU_SENTRY = 107,
    REF_BOT_BLU_DART = 108,
    REF_BOT_BLU_RADAR = 109,
    REF_BOT_BLU_OUTPOST = 111,
    REF_BOT_BLU_BASE = 110,
  };

  enum class ClientID : uint16_t {
    REF_CL_RED_HERO = 0x0101,
    REF_CL_RED_ENGINEER = 0x0102,
    REF_CL_RED_INFANTRY_1 = 0x0103,
    REF_CL_RED_INFANTRY_2 = 0x0104,
    REF_CL_RED_INFANTRY_3 = 0x0105,
    REF_CL_RED_DRONE = 0x0106,
    REF_CL_BLU_HERO = 0x0165,
    REF_CL_BLU_ENGINEER = 0x0166,
    REF_CL_BLU_INFANTRY_1 = 0x0167,
    REF_CL_BLU_INFANTRY_2 = 0x0168,
    REF_CL_BLU_INFANTRY_3 = 0x0169,
    REF_CL_BLU_DRONE = 0x016A,
    REF_CL_REFEREE_SERVER = 0x8080, /* 裁判系统服务器，用于哨兵和雷达自主决策 */
  };

  /**
   * @brief 子内容ID ,来源于0x0301
   *
   */
  enum class CMDID : uint16_t {
    REF_STDNT_CMD_ID_UI_DEL = 0x0100,     /*选手端删除图层*/
    REF_STDNT_CMD_ID_UI_DRAW1 = 0x0101,   /*选手端绘制一个图形*/
    REF_STDNT_CMD_ID_UI_DRAW2 = 0x0102,   /*选手端绘制两个图形*/
    REF_STDNT_CMD_ID_UI_DRAW5 = 0x0103,   /*选手端绘制五个图形*/
    REF_STDNT_CMD_ID_UI_DRAW7 = 0x0104,   /*选手端绘制七个图形*/
    REF_STDNT_CMD_ID_UI_STR = 0x0110,     /*选手端绘制字符图形*/
    REF_STDNT_CMD_ID_CUSTOM = 0x0200,     /*0x0200~0x02FF机器人之间通信*/
    REF_STDNT_CMD_ID_SENTRY_CMD = 0X0120, /*哨兵自主决策指令*/
    REF_STDNT_CMD_ID_RADAR_CMD = 0X0121,  /*雷达自主决策指令*/
    REF_STDNT_CMD_ID_SEND_RADAR_DATA = 0X0279 /*发送给雷达位置数据*/
  };

  /**
   * @brief 裁判系统的状态(可能会用到吧)
   *
   */
  enum class Status : uint8_t {
    OFFLINE = 0, /* 离线 */
    RUNNING,     /* 在线 */
  };

  /**
   * @brief 命令码定义，裁判系统更新写在这里
   *
   */
  enum class CommandID : uint16_t {
    REF_CMD_ID_GAME_STATUS = 0x0001, /* 比赛状态数据，固定以 1Hz 频率发送 */
    REF_CMD_ID_GAME_RESULT = 0x0002, /* 比赛结果数据，比赛结束触发发送 */
    REF_CMD_ID_GAME_ROBOT_HP = 0x0003, /* 机器人血量数据，固定以 3Hz 频率发送 */
    REF_CMD_ID_FIELD_EVENTS = 0x0101, /* 场地事件数据，固定以 1Hz 频率发送 */
    REF_CMD_ID_WARNING = 0x0104, /* 裁判警告数据，1Hz，触发时发送 */
    REF_CMD_ID_DART_COUNTDOWN = 0x0105, /* 飞镖发射相关数据，固定1Hz发送 */
    REF_CMD_ID_ROBOT_STATUS = 0x0201, /* 机器人性能体系数据，10Hz */
    REF_CMD_ID_POWER_HEAT_DATA = 0x0202, /* 实时底盘缓冲能量和射击热量, 10Hz */
    REF_CMD_ID_ROBOT_POS = 0x0203, /* 机器人位置数据, 1Hz */
    REF_CMD_ID_ROBOT_BUFF = 0x0204, /* 机器人增益和底盘能量数据, 3Hz */
    REF_CMD_ID_DRONE_ENERGY = 0x0205, /* 老💡的宝贝，不知道是啥 */
    REF_CMD_ID_ROBOT_DMG = 0x0206, /* 伤害状态数据，伤害发生后发送 */
    REF_CMD_ID_LAUNCHER_DATA = 0x0207, /* 实时射击数据，弹丸发射后发送 */
    REF_CMD_ID_BULLET_REMAINING = 0x0208, /* 允许发弹量, 10Hz */
    REF_CMD_ID_RFID = 0x0209,             /* 机器人RFID模块状态, 3Hz */
    REF_CMD_ID_DART_CLIENT = 0x020A,      /* 飞镖选手端指令数据, 3Hz */
    REF_CMD_ID_ROBOT_POS_TO_SENTRY = 0X020B, /* 地面机器人位置数据, 1Hz */
    REF_CMD_ID_RADAR_MARK = 0X020C,          /* 雷达标记进度数据, 1Hz */
    REF_CMD_ID_SENTRY_DECISION = 0x020D, /* 哨兵自主决策相关信息同步, 1Hz */
    REF_CMD_ID_RADAR_DECISION = 0x020E, /* 雷达自主决策相关信息同步, 1Hz */
    REF_CMD_ID_INTER_STUDENT = 0x0301, /* 机器人交互数据, 30Hz */
    REF_CMD_ID_INTER_STUDENT_CUSTOM = 0x0302, /*自定义控制器和机器人,30Hz,图传*/
    REF_CMD_ID_CLIENT_MAP = 0x0303, /* 选手端小地图交互数据, 选手触发发送 */
    REF_CMD_ID_KEYBOARD_MOUSE = 0x0304, /* 键鼠遥控数据， 30Hz, 图传链路 */
    REF_CMD_ID_MAP_ROBOT_DATA = 0x0305, /* 小地图接收雷达数据, 5HzMax */
    REF_CMD_ID_CUSTOM_KEYBOARD_MOUSE = 0X0306, /* 自定义控制器交互, 30Hz */
    REF_CMD_ID_SENTRY_POS_DATA = 0x0307, /* 小地图接收路径数据, 1Hz */
    REF_CMD_ID_ROBOT_POS_DATA = 0x0308, /* 选手端小地图接受机器人消息, 3Hz */

    REF_CMD_ID_CUSTOM_RECV_DATA = 0x0309, /* 自定义控制器收包,10Hz,图传链路 */
    REF_CMD_ID_DATA_TO_CUSTOM_CLIENT = 0x0310, /*自定义客户端,50Hz,图传链路*/

    REF_CMD_ID_SET_VIDEO_TRANS_CH = 0x0F01, /* 设置图传出图信道,应答,图传链路 */
    REF_CMD_ID_GET_VIDEO_TRANS_CH = 0x0F02, /* 查询当前出图信道,应答,图传链路 */

    REF_CMD_ID_RADAR_ENEMY_POS = 0x0A01, /*对方机器人的位置坐标,10Hz,雷达链路*/
    REF_CMD_ID_RADAR_ENEMY_HP = 0x0A02, /*对方机器人的血量信息,10Hz,雷达链路*/
    REF_CMD_ID_RADAR_ENEMY_BULLET = 0x0A03, /* 对方剩余发弹量,10Hz,雷达链路 */
    REF_CMD_ID_RADAR_ENEMY_INFO = 0x0A04, /*对方宏观状态信息,10Hz,雷达链路*/
    REF_CMD_ID_RADAR_ENEMY_BUFF = 0x0A05, /*对方增益效果,10Hz,雷达链路*/
    REF_CMD_ID_RADAR_ENEMY_KEY = 0x0A06, /*对方干扰波密钥,10Hz,雷达链路*/

  };

  /**
   * @brief 比赛模式，来源0x0001
   *
   */
  enum class GameType : uint8_t {
    REF_GAME_TYPE_RMUC = 1, /* 超抗 */
    REF_GAME_TYPE_RMUT,     /* 高校单项赛 */
    REF_GAME_TYPE_RMUA,     /* 人工智能挑战赛 */
    REF_GAME_TYPE_RMUL_3V3, /* 3V3 对抗*/
    REF_GAME_TYPE_RMUL_1V1, /* 步兵1v1 */
  };

  /**
   * @brief 当前比赛阶段，来源0x0001
   *
   */
  enum class GameProcess : uint8_t {
    REF_GAME_PROCESS_PENDING = 0,   /* 未开始 */
    REF_GAME_PROCESS_READY,         /* 准备阶段 */
    REF_GAME_PROCESS_SELF_TEST,     /* 裁判系统15s自检 */
    REF_GAME_PROCESS_COUNTING_DOWN, /* 5s倒计时 */
    REF_GAME_PROCESS_PROCESSING,    /* 正在进行 */
    REF_GAME_PROCESS_SCORING,       /* 结算中 */
  };

  /**
   * @brief 获胜方，来源0x0002
   *
   */
  enum class Winner : uint8_t {
    REF_GAME_WINNER_TIE = 0, /* 平局 */
    REF_GAME_WINNER_RED,     /* 红方胜利 */
    REF_GAME_WINNER_BLUE,    /* 蓝方胜利 */
  };

  /**
   * @brief 键盘数据，来源于0x0304
   *
   */
  enum class KeyBoard : uint16_t {
    KEY_W = 1U << 0,
    KEY_S = 1U << 1,
    KEY_A = 1U << 2,
    KEY_D = 1U << 3,
    KEY_SHIFT = 1U << 4,
    KEY_CTRL = 1U << 5,
    KEY_Q = 1U << 6,
    KEY_E = 1U << 7,
    KEY_R = 1U << 8,
    KEY_F = 1U << 9,
    KEY_G = 1U << 10,
    KEY_Z = 1U << 11,
    KEY_X = 1U << 12,
    KEY_C = 1U << 13,
    KEY_V = 1U << 14,
    KEY_B = 1U << 15,
  };

  /**
   * @brief UI 删除操作类型，来源子内容ID 0x0100
   *
   */
  enum class UIDeleteType : uint8_t {
    UI_DELETE_NO_OP = 0,
    UI_DELETE_LAYER = 1,
    UI_DELETE_ALL = 2,
  };

  /**
   * @brief UI 图形操作类型，来源子内容ID 0x0101
   *
   */
  enum class UIFigureOp : uint8_t {
    UI_OP_NO_OP = 0,
    UI_OP_ADD = 1,
    UI_OP_MODIFY = 2,
    UI_OP_DELETE = 3,
  };

  /**
   * @brief UI 图形类型，来源子内容ID 0x0101
   *
   */
  enum class UIFigureType : uint8_t {
    UI_TYPE_LINE = 0,
    UI_TYPE_RECT = 1,
    UI_TYPE_CIRCLE = 2,
    UI_TYPE_ELLIPSE = 3,
    UI_TYPE_ARC = 4,
    UI_TYPE_FLOAT = 5,
    UI_TYPE_INT = 6,
    UI_TYPE_CHAR = 7,
  };

  /**
   * @brief UI 图形颜色，来源子内容ID 0x0101
   *
   */
  enum class UIColor : uint8_t {
    UI_COLOR_TEAM = 0, /* 红/蓝，己方颜色 */
    UI_COLOR_YELLOW = 1,
    UI_COLOR_GREEN = 2,
    UI_COLOR_ORANGE = 3,
    UI_COLOR_PURPLE_RED = 4,
    UI_COLOR_PINK = 5,
    UI_COLOR_CYAN = 6,
    UI_COLOR_BLACK = 7,
    UI_COLOR_WHITE = 8,
  };

  /**
   * @brief UI 图层删除包，来源子内容ID 0x0100
   *
   */
  struct [[gnu::packed]] UILayerDelete {
    uint8_t delete_type; /* @enum UIDeleteType */
    uint8_t layer;       /* 0~9 */
  };

  /**
   * @brief UI 单图形定义，来源子内容ID 0x0101
   *
   */
  struct [[gnu::packed]] UIFigure {
    uint8_t figure_name[3];    /* 图形名，作为索引 */
    uint32_t operate_type : 3; /* @enum UIFigureOp */
    uint32_t figure_type : 3;  /* @enum UIFigureType */
    uint32_t layer : 4;        /* 图层号 0~9 */
    uint32_t color : 4;        /* @enum UIColor */
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
  };

  /**
   * @brief UI 双图形，来源子内容ID 0x0102
   *
   */
  struct [[gnu::packed]] UIFigure2 {
    UIFigure interaction_figure[2];
  };

  /**
   * @brief UI 五图形，来源子内容ID 0x0103
   *
   */
  struct [[gnu::packed]] UIFigure5 {
    UIFigure interaction_figure[5];
  };

  /**
   * @brief UI 七图形，来源子内容ID 0x0104
   *
   */
  struct [[gnu::packed]] UIFigure7 {
    UIFigure interaction_figure[7];
  };

  /**
   * @brief UI 字符图形，来源子内容ID 0x0110
   *
   */
  struct [[gnu::packed]] UICharacter {
    UIFigure grapic_data_struct; /* 与图形定义一致，figure_type应为字符 */
    uint8_t data[30];            /* ASCII/UTF-8 文本数据 */
  };

  static_assert(sizeof(UILayerDelete) == 2, "UI layer delete size mismatch");
  static_assert(sizeof(UIFigure) == 15, "UI figure size mismatch");
  static_assert(sizeof(UIFigure2) == 30, "UI figure2 size mismatch");
  static_assert(sizeof(UIFigure5) == 75, "UI figure5 size mismatch");
  static_assert(sizeof(UIFigure7) == 105, "UI figure7 size mismatch");
  static_assert(sizeof(UICharacter) == 45, "UI character size mismatch");

  /**
   * @brief 裁判系统包头
   *
   */
  struct [[gnu::packed]] Header {
    uint8_t sof = 0xA5;   /* 数据帧起始字节，固定值为 0xA5 */
    uint16_t data_length; /* 数据帧中 data 的长度 */
    uint8_t seq;          /* 包序号 */
    uint8_t crc8;         /* 包头crc8校验 */
  };

  /**
   * @brief 0x0001, 比赛状态数据，固定以 1Hz 频率发送
   *
   */
  struct [[gnu::packed]] GameStatus {
    uint8_t game_type : 4;      /* 比赛类型 */
    uint8_t game_progress : 4;  /* 当前比赛阶段 */
    uint16_t stage_remain_time; /* 当前阶段剩余时间 */
    uint64_t sync_time_stamp;   /* 时间戳 */
  };

  /**
   * @brief 0x0002,比赛结果数据，比赛结束触发发送
   *
   */
  struct [[gnu::packed]] GameResult {
    uint8_t winner; /* 获胜方 */
  };

  /**
   * @brief 0x0003,机器人血量数据，固定以3Hz频率发送
   *
   */
  struct [[gnu::packed]] RobotHP {
    uint16_t ally_1_robot_hp;  /* 己方1号英雄 */
    uint16_t reserved;         /* 2027 保留位 */
    uint16_t ally_3_robot_hp;  /* 己方3号步兵 */
    uint16_t ally_4_robot_hp;  /* 己方4号步兵 */
    int16_t damage_difference; /* 己方与对方全队总伤害之差 */
    uint16_t ally_7_robot_hp;  /* 己方7号哨兵 */
    uint16_t r_outpost_hp;     /* R 前哨站，固定颜色 */
    uint16_t ally_base_hp;     /* 己方基地 */
    uint16_t b_outpost_hp;     /* B 前哨站，固定颜色 */
    uint16_t enemy_base_hp;    /* 对方基地 */
    uint16_t ally_control_degree;
    uint16_t enemy_control_degree;
  };

  /**
   * @brief 0x0101 场地事件数据，固定以 1Hz 频率发送
   *
   */
  struct [[gnu::packed]] FieldEvents {
    uint32_t event_data;   /* 表 1-9 第一状态字，bit0-31 */
    uint32_t event_data_2; /* 表 1-9 第二状态字，bit32-63 */
  };

  /**
   * @brief 0x0104 裁判警告数据，1Hz，触发时发送
   *
   * @return typedef struct
   */
  struct [[gnu::packed]] Warning {
    uint8_t level;    /* 己方最后一次受到判罚的等级 */
    uint8_t robot_id; /*己方最后一次受到判罚的违规机器人ID*/
    uint8_t count; /*己方最后一次受到判罚的违规机器人对应判罚等级的违规次数*/
  };

  /**
   * @brief 0x0105 飞镖发射相关数据，固定1Hz发送
   *
   */
  struct [[gnu::packed]] DartCountdown {
    uint8_t countdown; /*己方飞镖发射剩余时间，单位：秒*/

    uint16_t dart_last_target : 3; /*bit0-2最近一次己方飞镖击中的目标*/
    uint16_t attack_count : 3; /*bit3-5对方最近被击中的目标累计被击中次数*/
    uint16_t dart_target : 2; /*bit6-7飞镖此时选定的击打目标*/
    uint16_t res : 8;         /* bit8-15保留位 */
  };

  /**
   * @brief 0x0201 机器人性能体系数据，10Hz
   *
   */
  struct [[gnu::packed]] RobotStatus {
    uint8_t robot_id;               /* 本机器人 ID */
    uint8_t robot_level;            /* 机器人等级 */
    uint16_t remain_hp;             /* 机器人当前血量 */
    uint16_t max_hp;                /* 机器人血量上限 */
    uint16_t shooter_cooling_value; /* 机器人射击热量每秒冷却值 */
    uint16_t shooter_heat_limit;    /* 机器人射击热量上限 */
    uint16_t chassis_power_limit;   /* 机器人底盘功率上限 */
    float bullet_speed_limit;       /* 机器人射击初速度上限 */
    uint8_t power_gimbal_output : 1; /* gimbal输出，0为无输出，1为24V输出 */
    uint8_t power_chassis_output : 1; /* chassis输出，0为无输出，1为24V输出*/
    uint8_t power_launcher_output : 1; /* shooter输出，0为无输出，1为24V 输出 */
  };

  /**
   * @brief 0x0202 实时底盘缓冲能量和射击热量, 10Hz
   *
   */
  struct [[gnu::packed]] PowerHeat {
    uint16_t res_1;                /* 保留位 */
    uint16_t res_2;                /* 保留位 */
    float res_3;                   /* 保留位 */
    uint16_t chassis_pwr_buff;     /* 缓冲能量 单位：J */
    uint16_t launcher_id1_17_heat; /* 第1个17mm发射机构的射击热量 */
    uint16_t launcher_42_heat;     /* 42mm发射机构的射击热量 */
  };

  /**
   * @brief 0x0203 机器人位置数据, 1Hz
   *
   */
  struct [[gnu::packed]] RobotPOS {
    float x;     /* 本机器人位置x坐标，单位m */
    float y;     /* 本机器人位置y坐标，单位m */
    float angle; /* 本机器人测速模块的朝向 单位：度 正北为0度 */
  };

  /**
   * @brief 0x0204 机器人增益和底盘能量数据, 3Hz
   *
   */
  struct [[gnu::packed]] RobotBuff {
    uint8_t healing_buff; /*机器人回血增益,百分比*/
    uint16_t cooling_acc; /*机器人射击热量冷却增益 单位是s^{-1}*/
    uint8_t defense_buff; /*机器人防御增益 百分比*/
    uint8_t vulnerability_buff; /*机器人负防御增益 百分比*/
    uint16_t attack_buff;       /*机器人攻击增益 百分比*/
  };

  /**
   * @brief 0x0206 伤害状态数据，伤害发生后发送
   *
   */
  struct [[gnu::packed]] RobotDamage {
    uint8_t armor_id : 4;    /* 受击打的装甲板id */
    uint8_t damage_type : 4; /* 血量变化类型 */
    uint16_t hit_sequence;
    int16_t hit_x;
    int16_t hit_y;
  };

  /**
   * @brief 0x0207 实时射击数据，弹丸发射后发送
   *
   */
  struct [[gnu::packed]] LauncherData {
    uint8_t bullet_type;   /* 弹丸类型 */
    uint8_t launcherer_id; /* 发射机构 ID */
    uint8_t bullet_freq;   /* 弹丸射速 Hz */
    float bullet_speed;    /* 弹丸初速度 m/s */
  };

  /**
   * @brief 0x0208 允许发弹量, 10Hz
   *
   */
  struct [[gnu::packed]] BulletRemain {
    uint16_t bullet_17_remain; /*  17mm 弹丸允许发弹量 */
    uint16_t bullet_42_remain; /*  42mm 弹丸允许发弹量 */
    uint16_t coin_remain;      /* 剩余金币数量 */
    uint16_t bullet_17_store; /* 堡垒增益点提供的储备 17mm 弹丸允许发弹量 */
  };

  /**
   * @brief 0x0209 机器人RFID模块状态, 3Hz
   *
   */
  struct [[gnu::packed]] RFID {
    uint32_t rfid_status;   /* bit 0-31 */
    uint16_t rfid_status_2; /* 表1-19 第二状态字，bit8 为隧道 RFID */
  };

  static_assert(sizeof(GameStatus) == 11, "0x0001 size mismatch");
  static_assert(sizeof(RobotHP) == 24, "0x0003 size mismatch");
  static_assert(sizeof(FieldEvents) == 8, "0x0101 size mismatch");
  static_assert(sizeof(RobotStatus) == 17, "0x0201 size mismatch");
  static_assert(sizeof(PowerHeat) == 14, "0x0202 size mismatch");
  static_assert(sizeof(RobotBuff) == 7, "0x0204 size mismatch");
  static_assert(sizeof(RobotDamage) == 7, "0x0206 size mismatch");
  static_assert(sizeof(BulletRemain) == 8, "0x0208 size mismatch");
  static_assert(sizeof(RFID) == 6, "0x0209 size mismatch");

  /**
   * @brief 0x020A 飞镖选手端指令数据, 3Hz
   *
   */
  struct [[gnu::packed]] DartClient {
    uint8_t opening_status;              /*当前飞镖发射站的状态*/
    uint8_t res;                         /*保留位*/
    uint16_t target_changable_countdown; /*切换目标时比赛剩余时间，单位秒*/
    uint16_t operator_cmd_launch_time; /*最后一次确定发射时的剩余时间，单位s*/
  };

  /**
   * @brief 0x020B 地面机器人位置数据, 1Hz
   *
   */
  struct [[gnu::packed]] RobotPosForSentry {
    float hero_x; /*己方英雄机器人位置 x 轴坐标，单位：m*/
    float hero_y; /*己方英雄机器人位置 y 轴坐标，单位：m*/
    float reserved_engineer_x; /*己方工程机器人位置 x 轴坐标，单位：m*/
    float reserved_engineer_y; /*己方工程机器人位置 y 轴坐标，单位：m*/
    float standard_3_x; /*己方 3 号步兵机器人位置 x 轴坐标，单位：m*/
    float standard_3_y; /*己方 3 号步兵机器人位置 y 轴坐标，单位：m*/
    float standard_4_x; /*己方 4 号步兵机器人位置 x 轴坐标，单位：m*/
    float standard_4_y; /*己方 4 号步兵机器人位置 y 轴坐标，单位：m*/
    float res_1;        /*保留位*/
    float res_2;        /*保留位*/
  };

  /**
   * @brief 0x020C 雷达标记进度数据, 1Hz
   *
   */
  struct [[gnu::packed]] RadarMarkProgress {
    uint16_t mark_progress;
  };

  /**
   * @brief 0x020D 哨兵自主决策相关信息同步, 1Hz
   *
   */
  struct [[gnu::packed]] SentryInfo {
    uint32_t info_1;
    uint16_t info_2;
    uint64_t info_3;

    uint8_t CurrentStance() const {
      return static_cast<uint8_t>((info_2 >> 12U) & 0x03U);
    }
  };
  static_assert(sizeof(SentryInfo) == 14U);

  /**
   * @brief 0x020E 雷达自主决策相关信息同步, 1Hz
   *
   */
  struct [[gnu::packed]] RadarInfo {
    uint8_t qualification : 2; /*雷达是否拥有触发双倍易伤的机会*/
    uint8_t enemy_status : 1;  /*对方是否正在被触发双倍易伤*/
    uint8_t encript_level : 2; /*己方加密等级（即对方干扰波难度等级）*/
    uint8_t could_exchange_key : 1; /*当前是否可以修改密钥*/
    uint8_t res : 2;                /*保留位*/
    uint8_t fortify_info;
    uint16_t fortify_center_x;
    uint16_t fortify_center_y;
    uint16_t fortify_radius;
  };

  /**
   * @brief 0x0301 机器人交互数据, 30Hz
   *
   */
  struct [[gnu::packed]] RobotInteractionData {
    CMDID data_cmd_id; /* 子内容ID,需为开放的子内容 ID */
    uint16_t sender_id; /*发送者 ID,需与自身 ID 匹配，ID 编号详见附录*/
    uint16_t receiver_id;  /*接收者 ID*/
    int8_t user_data[112]; /*内容数据段*/
  };

  /**
   * @brief 0x0302 自定义控制器和机器人,30Hz,图传链路;
   *       操作手可使用自定义控制器通过图传链路向对应的机器人发送数据
   *
   */
  struct [[gnu::packed]] CustomController {
    uint8_t data[30];
  };

  /**
   * @brief 0x0303 选手端小地图交互数据, 选手触发发送
   *
   */
  struct [[gnu::packed]] ClientMap {
    float position_x;       /*目标位置 x 轴坐标，单位 m*/
    float position_y;       /*目标位置 y 轴坐标，单位 m*/
    uint8_t commd_keyboard; /*云台手按下的键盘按键通用键值*/
    uint8_t robot_id;       /*对方机器人 ID*/
    uint16_t cmd_source;    /*信息来源 ID*/
  };

  /**
   * @brief 0x0304 键鼠遥控数据， 30Hz, 图传链路;
   *        通过遥控器发送的键鼠遥控数据将同步通过图传链路发送给对应机器人
   *
   */
  struct [[gnu::packed]] KeyboardMouse {
    int16_t mouse_x;     /*鼠标 x 轴移动速度，负值标识向左移动*/
    int16_t mouse_y;     /*鼠标 y 轴移动速度，负值标识向下移动*/
    int16_t mouse_wheel; /*鼠标滚轮移动速度，负值标识向后滚动*/
    int8_t button_l; /*鼠标左键是否按下：0 为未按下；1 为按下*/
    int8_t button_r; /*鼠标右键是否按下：0 为未按下，1 为按下*/
    uint16_t keyboard_value; /* 键盘按键信息 @enum KeyBoard */
    uint16_t res;            /*保留位*/
  };

  /**
   * @brief 0x0305 小地图接收雷达数据, 5HzMax
   *
   */
  struct [[gnu::packed]] MapRobotData {
    uint16_t enemy_hero_x, enemy_hero_y;
    uint16_t reserved_enemy_2_x, reserved_enemy_2_y;
    uint16_t enemy_infantry_3_x, enemy_infantry_3_y;
    uint16_t enemy_infantry_4_x, enemy_infantry_4_y;
    uint16_t reserved_enemy_5_x, reserved_enemy_5_y;
    uint16_t enemy_sentry_x, enemy_sentry_y;
    uint16_t ally_hero_x, ally_hero_y;
    uint16_t reserved_ally_2_x, reserved_ally_2_y;
    uint16_t ally_infantry_3_x, ally_infantry_3_y;
    uint16_t ally_infantry_4_x, ally_infantry_4_y;
    uint16_t reserved_ally_5_x, reserved_ally_5_y;
    uint16_t ally_sentry_x, ally_sentry_y;
  };

  /**
   * @brief 0x0306 自定义控制器交互, 30Hz;
   *        操作手可使用自定义控制器模拟键鼠操作选手端
   * @warning 若无新的按键信息，将保持上一帧数据的按下状态
   */
  struct [[gnu::packed]] CustomKeyMouseData {
    uint8_t key1_value;       /*按键 1 键值*/
    uint8_t key2_value;       /*按键 2 键值*/
    uint16_t x_position : 12; /*鼠标 X 轴像素位置*/
    uint16_t mouse_left : 4;  /*鼠标左键状态*/
    uint16_t y_position : 12; /*鼠标 Y 轴像素位置*/
    uint16_t mouse_right : 4; /*鼠标右键状态*/
    uint16_t res;             /*保留位*/
  };

  /**
   * @brief 0x0307 小地图接收路径数据, 1Hz
   *
   * @note 哨兵机器人或半自动控制方式的机器人可通过
   *       常规链路向对应的操作手选手端发送路径坐标
   *       数据，该路径会在小地图上显示
   */
  struct [[gnu::packed]] SentryPosition {
    uint8_t intention; /*移动到目标点或者到目标点攻击/防守*/
    uint16_t start_position_x; /*路径起点 x 轴坐标，单位：dm*/
    uint16_t start_position_y; /*路径起点 y 轴坐标，单位：dm*/
    int8_t delta_x[49];        /*路径点 x 轴增量数组，单位：dm*/
    int8_t delta_y[49];        /*路径点 y 轴增量数组，单位：dm*/
    uint16_t sender_id;        /*发送者 ID*/
  };

  /**
   * @brief 0x0308 选手端小地图接受机器人消息, 3Hz
   *
   * @note 编码发送时注意数据的大小端问题
   */
  struct [[gnu::packed]] RobotPosition {
    uint16_t sender_id;   /*发送者的 ID*/
    uint16_t receiver_id; /*接收者的 ID*/
    int8_t user_data[30]; /*字符，以 utf-16 格式编码发送，支持显示中文*/
  };

  /**
   * @brief 0x0309 自定义数据
   *        机器人可通过图传链路向对应的操作手选手端
   *        连接的自定义控制器发送数据(RMUL暂不适用)
   *
   */
  struct [[gnu::packed]] CustomData1 {
    uint8_t data[30]; /*自定义数据*/
  };

  /**
   * @brief 0x0310 自定义数据
   *        机器人可通过图传链路向对应的操作手选手端
   *        连接的自定义控制器发送数据(RMUL暂不适用)
   *
   */
  struct [[gnu::packed]] CustomData2 {
    uint8_t data[300]; /*表1-39完整自定义数据*/
  };

  /**
   * @brief 0x0A01 对方机器人的位置坐标
   *
   */
  struct [[gnu::packed]] RadarEnemyRobotPos {
    uint16_t enemy_hero_x; /*对方英雄机器人 x 位置坐标，单位：cm*/
    uint16_t enemy_hero_y; /*对方英雄机器人 y 位置坐标，单位：cm*/
    uint16_t reserved_engineer_x; /*对方工程机器人 x 位置坐标，单位：cm*/
    uint16_t reserved_engineer_y; /*对方工程机器人 y 位置坐标，单位：cm*/
    uint16_t enemy_infantry_3_x; /*对方3 号步兵 x 位置坐标，单位：cm*/
    uint16_t enemy_infantry_3_y; /*对方3 号步兵 y 位置坐标，单位：cm*/
    uint16_t enemy_infantry_4_x; /*对方4 号步兵 x 位置坐标，单位：cm*/
    uint16_t enemy_infantry_4_y; /*对方4 号步兵 y 位置坐标，单位：cm*/
    uint16_t reserved_drone_x; /*对方空中机器人 x 位置坐标，单位：cm*/
    uint16_t reserved_drone_y; /*对方空中机器人 y 位置坐标，单位：cm*/
    uint16_t enemy_sentry_x; /*对方哨兵机器人 x 位置坐标，单位：cm*/
    uint16_t enemy_sentry_y; /*对方哨兵机器人 y 位置坐标，单位：cm*/
  };

  /**
   * @brief 0x0A02 对方机器人的血量信息
   *
   */
  struct [[gnu::packed]] RadarEnemyRobotHP {
    uint16_t enemy_hero_hp;        /*对方 1 号英雄机器人血量*/
    uint16_t reserved_engineer_hp; /*对方 2 号工程机器人血量*/
    uint16_t enemy_infantry_3_hp;  /*对方 3 号步兵机器人血量*/
    uint16_t enemy_infantry_4_hp;  /*对方 4 号步兵机器人血量*/
    uint16_t res;                  /*保留位*/
    uint16_t enemy_sentry_hp;      /*对方 7 号哨兵机器人血量*/
  };

  /**
   * @brief 0x0A03 对方机器人的剩余发弹量信息
   *
   */
  struct [[gnu::packed]] RadarEnemyBullet {
    uint16_t enemy_hero_bullet;
    uint16_t enemy_infantry_3_bullet;
    uint16_t enemy_infantry_4_bullet;
    uint16_t reserved;
    uint16_t enemy_sentry_bullet;
  };

  /**
   * @brief 0x0A04 对方队伍的宏观状态信息
   *
   */
  struct [[gnu::packed]] RadarEnemyState {
    uint16_t enemy_remain_coins;
    uint16_t enemy_coins_total;
    uint32_t activation_status;
    uint32_t terrain_control_status;
  };

  /**
   * @brief 0x0A05 对方各机器人当前增益效果
   *
   */
  struct [[gnu::packed]] RadarRobotBuff {
    uint8_t enemy_hero_health; /*对方英雄机器人回血增益, 百分比*/
    uint16_t enemy_hero_heat; /* 对方英雄射击热量冷却增益,直接值,单位/s */
    uint8_t enemy_hero_protection; /*对方英雄机器人防御增益,百分比*/
    uint8_t enemy_hero_neg_protection; /*对方英雄机器人负防御增益,百分比*/
    uint16_t enemy_hero_attact; /*对方英雄机器人攻击增益,百分比*/

    uint8_t reserved_engineer[7];

    uint8_t enemy_infantry_3_health; /*对方3号步兵回血增益, 百分比*/
    uint16_t enemy_infantry_3_heat; /* 对方3号射击热量冷却增益,直接值,单位/s */
    uint8_t enemy_infantry_3_protection; /*对方3号步兵防御增益,百分比*/
    uint8_t enemy_infantry_3_neg_protection; /*对方3号步兵负防御增益,百分比*/
    uint16_t enemy_infantry_3_attact; /*对方3号步兵攻击增益,百分比*/

    uint8_t enemy_infantry_4_health; /*对方4号步兵回血增益, 百分比*/
    uint16_t enemy_infantry_4_heat; /* 对方4号射击热量冷却增益,直接值,单位/s */
    uint8_t enemy_infantry_4_protection; /*对方4号步兵防御增益,百分比*/
    uint8_t enemy_infantry_4_neg_protection; /*对方4号步兵负防御增益,百分比*/
    uint16_t enemy_infantry_4_attact; /*对方4号步兵攻击增益,百分比*/

    uint8_t enemy_sentry_health; /*对方哨兵回血增益, 百分比*/
    uint16_t enemy_sentry_heat; /* 对方哨兵射击热量冷却增益,直接值,单位/s */
    uint8_t enemy_sentry_protection;     /*对方哨兵防御增益,百分比*/
    uint8_t enemy_sentry_neg_protection; /*对方哨兵负防御增益,百分比*/
    uint16_t enemy_sentry_attact;        /*对方哨兵攻击增益,百分比*/
    uint8_t enemy_sentry_stance;
    uint8_t enemy_hero_state;
    uint8_t reserved_state;
    uint8_t enemy_infantry_3_state;
    uint8_t enemy_infantry_4_state;
    uint8_t enemy_sentry_state;
    uint16_t fortify_center_x;
    uint16_t fortify_center_y;
    uint16_t fortify_radius;
  };

  /**
   * @brief 0x0A06 对方干扰波密钥
   *
   */
  struct [[gnu::packed]] RadarEnemyKey {
    uint8_t ascii[6]; /*每个字节均为 ASCII 码编码的字母或数字*/
  };

  /**
   * @brief 0x0120 哨兵自主决策指令
   *
   */
  struct [[gnu::packed]] SentryDecisionData {
    uint32_t confirm_resurrection : 1; /* 哨兵机器人是否确认复活 */
    uint32_t buy_resurrection : 1; /* 哨兵机器人是否确认兑换立即复活 */
    uint32_t buy_bullet_num : 11; /* 哨兵将要兑换的发弹量值 */
    uint32_t remote_buy_bullet_times : 4; /* 哨兵远程兑换发弹量的请求次数 */
    uint32_t romote_buy_hp_times : 4; /* 哨兵远程兑换血量的请求次数 */
    uint32_t current_state : 3; /* 哨兵修改当前姿态指令,1进攻 2防御 3移动 */
    uint32_t comfirm_mech : 1; /*哨兵机器人是否确认使能量机关进入正在激活状态*/
    uint32_t res : 7; /* 保留位 */
  };
  static_assert(sizeof(SentryDecisionData) == 4U);

  /**
   * @brief 0x0121 雷达自主决策指令
   *
   */
  struct [[gnu::packed]] RadarDecisionData {
    uint8_t radar_cmd; /* 雷达是否确认触发双倍易伤（单调+1） */
    uint8_t password_cmd; /* byte1：1改密钥；2提交破解密钥验证 */
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
    uint8_t armor_fortify_cmd;
    uint16_t fortify_center_x;
    uint16_t fortify_center_y;
  };

  static_assert(sizeof(RadarInfo) == 8);
  static_assert(sizeof(RadarDecisionData) == 13);
  static_assert(sizeof(ClientMap) == 12);
  static_assert(sizeof(MapRobotData) == 48);
  static_assert(sizeof(CustomData2) == 300);
  static_assert(sizeof(RadarEnemyBullet) == 10);
  static_assert(sizeof(RadarEnemyState) == 12);
  static_assert(sizeof(RadarRobotBuff) == 47);

  /**
   * @brief 0x0F01 设置图传出图信道，应答
   *
   */
  struct [[gnu::packed]] SetVideoTransChannel {
    uint8_t value; /* 请求:1~6; 应答:0成功/1忙/2参数错误 */
  };

  /**
   * @brief 0x0F02 查询图传出图信道
   *
   * @note 查询请求不需要数据段；应答为1字节当前信道
   */
  struct [[gnu::packed]] GetVideoTransChannel {
    uint8_t channel; /* 0表示未设置；1~6为当前信道 */
  };

  /**
   * @brief 传递的总结构体,共27条
   *
   */
  struct Data {
    Status status = Status::OFFLINE;            /* 在线状态 */
    GameStatus game_status;                     /* 0x0001 */
    GameResult game_result;                     /* 0x0002 */
    RobotHP game_robot_hp;                      /* 0x0003 */
    FieldEvents field_event;                    /* 0x0101 */
    Warning warning;                            /* 0x0104 */
    DartCountdown dart_countdown;               /* 0x0105 */
    RobotStatus robot_status;                   /* 0x0201 */
    PowerHeat power_heat;                       /* 0x0202 */
    RobotPOS robot_pos;                         /* 0x0203 */
    RobotBuff robot_buff;                       /* 0x0204 */
    RobotDamage robot_damage;                   /* 0x0206 */
    LauncherData launcher_data;                 /* 0x0207 */
    BulletRemain bullet_remain;                 /* 0x0208 */
    RFID rfid;                                  /* 0x0209 */
    DartClient dart_client;                     /* 0x020A */
    RobotPosForSentry sentry_pos;               /* 0x020B */
    RadarMarkProgress radar_mark_progress;      /* 0x020C */
    SentryInfo sentry_decision;                 /* 0x020D */
    RadarInfo radar_decision;                   /* 0x020E */
    RobotInteractionData robot_ineraction_data; /* 0x0301 */
    CustomController custom_controller;         /* 0x0302 */
    ClientMap client_map;                       /* 0x0303 */
    KeyboardMouse keyboard_mouse;               /* 0x0304 */
    MapRobotData map_robot_data;                /* 0x0305 */
    CustomKeyMouseData custom_key_mouse_data;   /* 0x0306 */
    SentryPosition sentry_postion;              /* 0x0307 */
    RobotPosition robot_position;               /* 0x0308 */
    CustomData1 custom_data1;                   /* 0x0309 */
    CustomData2 custom_data2;                   /* 0x0310 */
    RadarEnemyRobotPos radar_enemy_robot_pos;   /* 0x0A01 */
    RadarEnemyRobotHP radar_enemy_robot_hp;     /* 0x0A02 */
    RadarEnemyBullet radar_enemy_bullet;        /* 0x0A03 */
    RadarEnemyState radar_enemy_state;          /* 0x0A04 */
    RadarRobotBuff radar_robot_buff;            /* 0x0A05 */
    RadarEnemyKey radar_enemy_key;              /* 0x0A06 */
    SentryDecisionData sentry_dec_data;         /* 0x0120 */
    RadarDecisionData radar_dec_data;           /* 0x0121 */
    SetVideoTransChannel set_vid_trans_ch;      /* 0x0F01 */
    GetVideoTransChannel get_vid_trans_ch;      /* 0x0F02 */
  };

  /**
   * @brief 发射模块需要的裁判系统数据
   *
   */
  struct [[gnu::packed]] LauncherPack {
    RobotStatus rs;  /* 热量上限和冷却速率 */
    RobotBuff rb;    /* 冷却增益 */
    LauncherData ld; /* 射速 */
    DartClient dc;   /* 飞镖发射 */
    PowerHeat ph;
  };

  /**
   * @brief 底盘模块需要的包
   *
   */
  struct [[gnu::packed]] ChassisPack {
    RobotStatus rs;        /* 等级和功率上限 */
    uint16_t power_buffer; /* 底盘缓冲能量，单位 J */
    uint32_t robot_status_received_time_ms = 0U; /* 0x0201 接收时间 */
    bool robot_status_received = false;          /* 已收到有效 0x0201 */
    uint32_t power_heat_received_time_ms = 0U;   /* 0x0202 接收时间 */
    bool power_heat_received = false;            /* 已收到有效 0x0202 */
  };

  /**
   * @brief 哨兵姿态
   *
   */
  enum class State : uint8_t {
    ATTACH = 1,
    DEFEND = 2,
    GUERRILLA = 3,
  };

  static constexpr uint16_t SOURCE_GAME_STATUS = 1U << 0;
  static constexpr uint16_t SOURCE_ROBOT_HP = 1U << 1;
  static constexpr uint16_t SOURCE_FIELD_EVENT = 1U << 2;
  static constexpr uint16_t SOURCE_ROBOT_STATUS = 1U << 3;
  static constexpr uint16_t SOURCE_POWER_HEAT = 1U << 4;
  static constexpr uint16_t SOURCE_ROBOT_POS = 1U << 5;
  static constexpr uint16_t SOURCE_SENTRY_POS = 1U << 10;
  static constexpr uint16_t SOURCE_SENTRY_INFO = 1U << 11;
  static constexpr uint16_t SOURCE_ROBOT_BUFF = 1U << 6;
  static constexpr uint16_t SOURCE_ROBOT_DAMAGE = 1U << 7;
  static constexpr uint16_t SOURCE_BULLET_REMAIN = 1U << 8;
  static constexpr uint16_t SOURCE_RFID = 1U << 9;
  static constexpr uint16_t SUPPORTED_SOURCE_MASK =
      SOURCE_GAME_STATUS | SOURCE_ROBOT_HP | SOURCE_FIELD_EVENT |
      SOURCE_ROBOT_STATUS | SOURCE_POWER_HEAT | SOURCE_ROBOT_POS |
      SOURCE_ROBOT_BUFF | SOURCE_ROBOT_DAMAGE | SOURCE_BULLET_REMAIN |
      SOURCE_RFID | SOURCE_SENTRY_POS | SOURCE_SENTRY_INFO;
  using SourceTimes = std::array<uint32_t, 12>;

  static uint16_t SourceMaskForCommand(CommandID id) {
    switch (id) {
      case CommandID::REF_CMD_ID_GAME_STATUS:
        return SOURCE_GAME_STATUS;
      case CommandID::REF_CMD_ID_GAME_ROBOT_HP:
        return SOURCE_ROBOT_HP;
      case CommandID::REF_CMD_ID_FIELD_EVENTS:
        return SOURCE_FIELD_EVENT;
      case CommandID::REF_CMD_ID_ROBOT_STATUS:
        return SOURCE_ROBOT_STATUS;
      case CommandID::REF_CMD_ID_POWER_HEAT_DATA:
        return SOURCE_POWER_HEAT;
      case CommandID::REF_CMD_ID_ROBOT_POS:
        return SOURCE_ROBOT_POS;
      case CommandID::REF_CMD_ID_ROBOT_POS_TO_SENTRY:
        return SOURCE_SENTRY_POS;
      case CommandID::REF_CMD_ID_SENTRY_DECISION:
        return SOURCE_SENTRY_INFO;
      case CommandID::REF_CMD_ID_ROBOT_BUFF:
        return SOURCE_ROBOT_BUFF;
      case CommandID::REF_CMD_ID_ROBOT_DMG:
        return SOURCE_ROBOT_DAMAGE;
      case CommandID::REF_CMD_ID_BULLET_REMAINING:
        return SOURCE_BULLET_REMAIN;
      case CommandID::REF_CMD_ID_RFID:
        return SOURCE_RFID;
      default:
        return 0U;
    }
  }

  // -1 is the bounded 0x0301 payload, -2 means unsupported/legacy.
  static int ExpectedPayloadSize(CommandID id) {
    switch (id) {
      case CommandID::REF_CMD_ID_GAME_STATUS:
        return sizeof(GameStatus);
      case CommandID::REF_CMD_ID_GAME_RESULT:
        return sizeof(GameResult);
      case CommandID::REF_CMD_ID_GAME_ROBOT_HP:
        return sizeof(RobotHP);
      case CommandID::REF_CMD_ID_FIELD_EVENTS:
        return sizeof(FieldEvents);
      case CommandID::REF_CMD_ID_WARNING:
        return sizeof(Warning);
      case CommandID::REF_CMD_ID_DART_COUNTDOWN:
        return sizeof(DartCountdown);
      case CommandID::REF_CMD_ID_ROBOT_STATUS:
        return sizeof(RobotStatus);
      case CommandID::REF_CMD_ID_POWER_HEAT_DATA:
        return sizeof(PowerHeat);
      case CommandID::REF_CMD_ID_ROBOT_POS:
        return sizeof(RobotPOS);
      case CommandID::REF_CMD_ID_ROBOT_BUFF:
        return sizeof(RobotBuff);
      case CommandID::REF_CMD_ID_ROBOT_DMG:
        return sizeof(RobotDamage);
      case CommandID::REF_CMD_ID_LAUNCHER_DATA:
        return sizeof(LauncherData);
      case CommandID::REF_CMD_ID_BULLET_REMAINING:
        return sizeof(BulletRemain);
      case CommandID::REF_CMD_ID_RFID:
        return sizeof(RFID);
      case CommandID::REF_CMD_ID_DART_CLIENT:
        return sizeof(DartClient);
      case CommandID::REF_CMD_ID_ROBOT_POS_TO_SENTRY:
        return sizeof(RobotPosForSentry);
      case CommandID::REF_CMD_ID_RADAR_MARK:
        return sizeof(RadarMarkProgress);
      case CommandID::REF_CMD_ID_SENTRY_DECISION:
        return sizeof(SentryInfo);
      case CommandID::REF_CMD_ID_RADAR_DECISION:
        return sizeof(RadarInfo);
      case CommandID::REF_CMD_ID_INTER_STUDENT_CUSTOM:
        return sizeof(CustomController);
      case CommandID::REF_CMD_ID_CLIENT_MAP:
        return sizeof(ClientMap);
      case CommandID::REF_CMD_ID_KEYBOARD_MOUSE:
        return sizeof(KeyboardMouse);
      case CommandID::REF_CMD_ID_MAP_ROBOT_DATA:
        return sizeof(MapRobotData);
      case CommandID::REF_CMD_ID_CUSTOM_KEYBOARD_MOUSE:
        return sizeof(CustomKeyMouseData);
      case CommandID::REF_CMD_ID_SENTRY_POS_DATA:
        return sizeof(SentryPosition);
      case CommandID::REF_CMD_ID_ROBOT_POS_DATA:
        return sizeof(RobotPosition);
      case CommandID::REF_CMD_ID_DATA_TO_CUSTOM_CLIENT:
        return sizeof(CustomData2);
      case CommandID::REF_CMD_ID_RADAR_ENEMY_POS:
        return sizeof(RadarEnemyRobotPos);
      case CommandID::REF_CMD_ID_RADAR_ENEMY_HP:
        return sizeof(RadarEnemyRobotHP);
      case CommandID::REF_CMD_ID_RADAR_ENEMY_BULLET:
        return sizeof(RadarEnemyBullet);
      case CommandID::REF_CMD_ID_RADAR_ENEMY_INFO:
        return sizeof(RadarEnemyState);
      case CommandID::REF_CMD_ID_RADAR_ENEMY_BUFF:
        return sizeof(RadarRobotBuff);
      case CommandID::REF_CMD_ID_RADAR_ENEMY_KEY:
        return sizeof(RadarEnemyKey);
      case CommandID::REF_CMD_ID_INTER_STUDENT:
        return -1;
      default:
        return -2;
    }
  }

  static bool ValidInteraction(const uint8_t* payload, uint16_t length) {
    if (payload == nullptr || length < 6U || length > 118U) return false;
    const uint16_t id = payload[0] | static_cast<uint16_t>(payload[1] << 8U);
    const size_t content = length - 6U;
    if (id == 0x0120U && content == sizeof(SentryDecisionData)) {
      const uint8_t stance = (payload[8] >> 5U) & 7U;
      return stance >= 1U && stance <= 3U;
    }
    if (id == 0x0121U && content == sizeof(RadarDecisionData)) {
      if (payload[7] > 2U) return false;
      if (payload[7] != 0U) {
        for (size_t i = 8U; i < 14U; ++i)
          if (!((payload[i] >= '0' && payload[i] <= '9') ||
                (payload[i] >= 'a' && payload[i] <= 'z') ||
                (payload[i] >= 'A' && payload[i] <= 'Z')))
            return false;
      }
      return true;
    }
    if (id >= 0x0200U && id <= 0x02ffU) return true;
    switch (id) {
      case 0x0100:
        return content == sizeof(UILayerDelete);
      case 0x0101:
        return content == sizeof(UIFigure);
      case 0x0102:
        return content == sizeof(UIFigure2);
      case 0x0103:
        return content == sizeof(UIFigure5);
      case 0x0104:
        return content == sizeof(UIFigure7);
      case 0x0110:
        return content == sizeof(UICharacter);
      case 0x0120:
        return content == sizeof(SentryDecisionData);
      case 0x0121:
        return content == sizeof(RadarDecisionData);
      default:
        return false;
    }
  }

  static uint16_t FreshSourceMask(uint16_t valid_mask,
                                  const SourceTimes& received_ms,
                                  uint32_t now_ms) {
    uint16_t fresh = 0U;
    for (size_t index = 0; index < received_ms.size(); ++index) {
      const uint16_t bit = static_cast<uint16_t>(1U << index);
      const uint32_t timeout =
          (bit & (SOURCE_GAME_STATUS | SOURCE_FIELD_EVENT | SOURCE_ROBOT_POS |
                  SOURCE_SENTRY_POS | SOURCE_SENTRY_INFO))
              ? 3000U
              : 1000U;
      if ((valid_mask & bit) && now_ms - received_ms[index] <= timeout)
        fresh |= bit;
    }
    return fresh;
  }

  /**
   * @brief 机器人、比赛和发射相关的裁判系统摘要
   *
   */
  struct [[gnu::packed]] RobotGameRefereePack {
    RobotStatus robot_status;
    GameStatus game_status;
    RobotHP robot_hp;
    FieldEvents field_event;
    PowerHeat power_heat;
    RobotPOS robot_pos;
    RobotBuff robot_buff;
    RobotDamage robot_damage;
    BulletRemain bullet_remain;
    RFID rfid;
    SentryInfo sentry_info;
    RobotPosForSentry sentry_pos;
    uint16_t source_command_id = 0U;
    uint16_t source_valid_mask = 0U;
    uint16_t source_fresh_mask = 0U;
    bool referee_online = false;
    uint32_t robot_hp_received_time_ms = 0U;
    uint32_t sentry_pos_received_time_ms = 0U;
  };

  /**
   * @brief 哨兵包定义
   *
   */
  struct [[gnu::packed]] RadarPack {
    /* TODO: 待更新 */
    Referee::RobotPosForSentry rf; /* 地面机器人位置数据 */
    float x;                       /* 本机器人位置x坐标，单位m */
    float y;                       /* 本机器人位置y坐标，单位m */
  };

  /**
   * @brief 增加哨兵本次要兑换的发弹量
   */
  LibXR::ErrorCode AddNeedBullet(uint16_t bullet_delta) {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    if (!decision_baseline_) return LibXR::ErrorCode::STATE_ERR;
    if (bullet_delta == 0U) return LibXR::ErrorCode::ARG_ERR;
    const uint32_t total = data_.sentry_dec_data.buy_bullet_num + bullet_delta;
    if (total > MAX_BUY_BULLET_NUM) return LibXR::ErrorCode::OUT_OF_RANGE;
    data_.sentry_dec_data.buy_bullet_num = total;
    decision_pending_ = true;
    return LibXR::ErrorCode::OK;
  }

  void SetConfirmRevival(bool revival) {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    if (data_.sentry_dec_data.confirm_resurrection != revival)
      decision_pending_ = true;
    data_.sentry_dec_data.confirm_resurrection = revival;
  }

  LibXR::ErrorCode RequestRemoteBulletExchange() {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    if (!decision_baseline_) return LibXR::ErrorCode::STATE_ERR;
    if (confirmed_remote_bullet_ + pending_remote_bullet_ >= 15U)
      return LibXR::ErrorCode::OUT_OF_RANGE;
    ++pending_remote_bullet_;
    data_.sentry_dec_data.remote_buy_bullet_times =
        confirmed_remote_bullet_ + 1U;
    decision_pending_ = true;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode SetHPRemote() {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    if (!decision_baseline_) return LibXR::ErrorCode::STATE_ERR;
    if (confirmed_remote_hp_ + pending_remote_hp_ >= 15U)
      return LibXR::ErrorCode::OUT_OF_RANGE;
    ++pending_remote_hp_;
    data_.sentry_dec_data.romote_buy_hp_times = confirmed_remote_hp_ + 1U;
    decision_pending_ = true;
    return LibXR::ErrorCode::OK;
  }

  void SetRevivalRemote(bool revival) {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    data_.sentry_dec_data.buy_resurrection = revival;
    decision_pending_ = true;
  }

  LibXR::ErrorCode SetSwitchMode(State state) {
    const auto value = static_cast<uint8_t>(state);
    if (value < 1U || value > 3U) return LibXR::ErrorCode::ARG_ERR;
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    data_.sentry_dec_data.current_state = value;
    decision_pending_ = true;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode SendSentryPack() {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    if (!ValidRobotID(data_.robot_status.robot_id))
      return LibXR::ErrorCode::STATE_ERR;
    decision_pending_ = true;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode SendRadarPack() {
    RadarPack payload{};
    {
      LibXR::Mutex::LockGuard lock(tx_data_mutex_);
      payload = this->radar_pack_;
    }

    return SendStudentCmd(CMDID::REF_STDNT_CMD_ID_SEND_RADAR_DATA, GetRobotID(),
                          GetRobotID() + 2, payload);
  }

  /**
   * @brief Construct a new Referee object
   *
   * @param task_stack_depth_uart
   * @param uart
   * @param baudrate
   */
  Referee(LibXR::UART& uart, CMD* cmd, uint32_t task_stack_depth_uart = 2048,
          uint32_t baudrate = 115200,
          const char* referee_chassis_tp_name = "chassis_ref",
          const char* referee_launcher_tp_name = "launcher_ref",
          const char* referee_robot_game_tp_name = "robot_game_ref",
          const char* referee_radar_tp_name = "radar_ref",
          LibXR::Thread::Priority thread_priority_uart =
              LibXR::Thread::Priority::LOW)

      : uart_(&uart),
        sem_(0),
        op_(sem_, REFEREE_RX_TIMEOUT_MS),
        sem_tx_(),
        tx_callback_(LibXR::WriteOperation::Callback::Create(
            [](bool, Referee* self, LibXR::ErrorCode result) {
              self->tx_completion_.store(result, std::memory_order_relaxed);
              self->tx_finished_.store(true, std::memory_order_release);
            },
            this)),
        tx_op_(tx_callback_),
        cmd_(cmd),
        chassispack_topic_(LibXR::Topic::CreateTopic<ChassisPack>(
            referee_chassis_tp_name, nullptr, true)),
        launcherpack_topic_(LibXR::Topic::CreateTopic<LauncherPack>(
            referee_launcher_tp_name, nullptr, true)),
        robot_game_referee_topic_(
            LibXR::Topic::CreateTopic<RobotGameRefereePack>(
                referee_robot_game_tp_name, nullptr, true)),
        radar_pack_topic_(LibXR::Topic::CreateTopic<RadarPack>(
            referee_radar_tp_name, nullptr, true)) {
    video_link_ = baudrate == 921600U;
    data_.sentry_dec_data.current_state = 3U;
    uart_->SetConfig({baudrate, LibXR::UART::Parity::NO_PARITY, 8, 1});

    this->thread_.Create(this, ThreadFunc, "Referee", task_stack_depth_uart,
                         thread_priority_uart);
  }

  // Initialization only, before the control task starts.
  void BindCMD(CMD& cmd) {
    REQUIRE(cmd_ == nullptr || cmd_ == &cmd);
    if (cmd_ != nullptr) return;
    cmd_ = &cmd;
  }

  // OK means admitted to the bounded queue; completion errors remain visible
  // through GetLastTxError(), and the same frame is retained for retry.
  LibXR::ErrorCode SendFrame(CommandID cmd_id, const void* payload,
                             uint16_t PAYLOAD_LEN) {
    const auto id = static_cast<uint16_t>(cmd_id);
    const auto* bytes = static_cast<const uint8_t*>(payload);
    const int expected = ExpectedPayloadSize(cmd_id);
    if (expected == -2) return LibXR::ErrorCode::NOT_SUPPORT;
    if (PAYLOAD_LEN > sizeof(PendingFrame::payload) ||
        (PAYLOAD_LEN > 0 && payload == nullptr))
      return LibXR::ErrorCode::ARG_ERR;
    if ((expected >= 0 && PAYLOAD_LEN != expected) ||
        (expected == -1 && !ValidInteraction(bytes, PAYLOAD_LEN)))
      return LibXR::ErrorCode::SIZE_ERR;
    if (id == 0x0310U && !video_link_) return LibXR::ErrorCode::STATE_ERR;
    if (id == 0x0301U) {
      const uint16_t sender = bytes[2] | static_cast<uint16_t>(bytes[3] << 8U);
      LibXR::Mutex::LockGuard lock(tx_data_mutex_);
      if (!ValidRobotID(data_.robot_status.robot_id) ||
          sender != data_.robot_status.robot_id)
        return LibXR::ErrorCode::STATE_ERR;
    }
    PendingFrame frame{};
    frame.command = cmd_id;
    frame.length = PAYLOAD_LEN;
    if (PAYLOAD_LEN != 0)
      LibXR::Memory::FastCopy(frame.payload, payload, PAYLOAD_LEN);
    const auto result = tx_queue_.Push(frame);
    if (result != LibXR::ErrorCode::OK) tx_admission_errors_.fetch_add(1);
    return result;
  }

  // Retained API name; admission uses the same shared rate limiter as
  // SendFrame.
  LibXR::ErrorCode SendFrameLocked(CommandID cmd_id, const void* payload,
                                   uint16_t length) {
    return SendFrame(cmd_id, payload, length);
  }

  LibXR::ErrorCode GetLastTxError() const {
    return tx_completion_.load(std::memory_order_acquire);
  }
  uint32_t GetTxAdmissionErrors() const { return tx_admission_errors_.load(); }

  template <typename PayloadType>
  LibXR::ErrorCode SendFrame(CommandID cmd_id, const PayloadType& payload) {
    return SendFrame(cmd_id, &payload,
                     static_cast<uint16_t>(sizeof(PayloadType)));
  }

  template <typename PayloadType>
  LibXR::ErrorCode SendStudentCmd(CMDID data_cmd_id, uint16_t sender_id,
                                  uint16_t receiver_id,
                                  const PayloadType& payload) {
    if constexpr (sizeof(PayloadType) > 112U) return LibXR::ErrorCode::SIZE_ERR;
    struct [[gnu::packed]] {
      uint16_t data_cmd_id;
      uint16_t sender_id;
      uint16_t receiver_id;
      PayloadType payload;
    } interaction_pack{};
    interaction_pack.data_cmd_id = static_cast<uint16_t>(data_cmd_id);
    interaction_pack.sender_id = sender_id;
    interaction_pack.receiver_id = receiver_id;
    interaction_pack.payload = payload;
    return SendFrame(CommandID::REF_CMD_ID_INTER_STUDENT, &interaction_pack,
                     static_cast<uint16_t>(sizeof(interaction_pack)));
  }

  uint16_t GetRobotID() const {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    return data_.robot_status.robot_id;
  }

  Data GetReceivedData() const {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    return data_;
  }

  static void InitFigure(UIFigure& fig, const char* name, UIFigureOp op,
                         UIFigureType type, uint8_t layer, UIColor color,
                         uint16_t width, uint16_t x, uint16_t y) {
    fig = {};
    SetFigureName(fig.figure_name, name);
    fig.operate_type = static_cast<uint32_t>(op);
    fig.figure_type = static_cast<uint32_t>(type);
    fig.layer = layer;
    fig.color = static_cast<uint32_t>(color);
    fig.width = width;
    fig.start_x = x;
    fig.start_y = y;
  }

  static void SetFigureValue32(UIFigure& fig, int32_t value) {
    const uint32_t RAW = static_cast<uint32_t>(value);
    fig.details_c = RAW & 0x3FFu;
    fig.details_d = (RAW >> 10) & 0x7FFu;
    fig.details_e = (RAW >> 21) & 0x7FFu;
  }

  void FillCharacter(Referee::UICharacter& fig, const char* name,
                     Referee::UIFigureOp op, uint8_t layer,
                     Referee::UIColor color, uint16_t font_size, uint16_t width,
                     uint16_t x, uint16_t y, const char* text) {
    fig = {};
    SetFigureName(fig.grapic_data_struct.figure_name, name);
    fig.grapic_data_struct.operate_type = static_cast<uint32_t>(op);
    fig.grapic_data_struct.figure_type =
        static_cast<uint32_t>(Referee::UIFigureType::UI_TYPE_CHAR);
    fig.grapic_data_struct.layer = layer;
    fig.grapic_data_struct.color = static_cast<uint32_t>(color);
    fig.grapic_data_struct.details_a = font_size;

    uint16_t text_len = 0;
    if (text != nullptr) {
      text_len = static_cast<uint16_t>(strnlen(text, sizeof(fig.data)));
      LibXR::Memory::FastCopy(fig.data, text, text_len);
    }
    fig.grapic_data_struct.details_b = text_len;
    fig.grapic_data_struct.width = width;
    fig.grapic_data_struct.start_x = x;
    fig.grapic_data_struct.start_y = y;
  }

  void FillLine(Referee::UIFigure& fig, const char* name,
                Referee::UIFigureOp op, uint8_t layer, Referee::UIColor color,
                uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2,
                uint16_t y2) {
    InitFigure(fig, name, op, Referee::UIFigureType::UI_TYPE_LINE, layer, color,
               width, x1, y1);
    fig.details_d = x2;
    fig.details_e = y2;
  }

  void FillRect(Referee::UIFigure& fig, const char* name,
                Referee::UIFigureOp op, uint8_t layer, Referee::UIColor color,
                uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2,
                uint16_t y2) {
    InitFigure(fig, name, op, Referee::UIFigureType::UI_TYPE_RECT, layer, color,
               width, x1, y1);
    fig.details_d = x2;
    fig.details_e = y2;
  }

  void FillCircle(Referee::UIFigure& fig, const char* name,
                  Referee::UIFigureOp op, uint8_t layer, Referee::UIColor color,
                  uint16_t width, uint16_t x, uint16_t y, uint16_t radius) {
    InitFigure(fig, name, op, Referee::UIFigureType::UI_TYPE_CIRCLE, layer,
               color, width, x, y);
    fig.details_c = radius;
  }

  void FillEllipse(Referee::UIFigure& fig, const char* name,
                   Referee::UIFigureOp op, uint8_t layer,
                   Referee::UIColor color, uint16_t width, uint16_t x,
                   uint16_t y, uint16_t x_half_axis, uint16_t y_half_axis) {
    InitFigure(fig, name, op, Referee::UIFigureType::UI_TYPE_ELLIPSE, layer,
               color, width, x, y);
    fig.details_d = x_half_axis;
    fig.details_e = y_half_axis;
  }

  void FillArc(Referee::UIFigure& fig, const char* name, Referee::UIFigureOp op,
               uint8_t layer, Referee::UIColor color, uint16_t width,
               uint16_t x, uint16_t y, uint16_t start_angle, uint16_t end_angle,
               uint16_t x_half_axis, uint16_t y_half_axis) {
    InitFigure(fig, name, op, Referee::UIFigureType::UI_TYPE_ARC, layer, color,
               width, x, y);
    fig.details_a = start_angle;
    fig.details_b = end_angle;
    fig.details_d = x_half_axis;
    fig.details_e = y_half_axis;
  }

  void FillFloat(Referee::UIFigure& fig, const char* name,
                 Referee::UIFigureOp op, uint8_t layer, Referee::UIColor color,
                 uint16_t font_size, uint16_t width, uint16_t x, uint16_t y,
                 float value) {
    InitFigure(fig, name, op, Referee::UIFigureType::UI_TYPE_FLOAT, layer,
               color, width, x, y);
    fig.details_a = font_size;
    SetFigureValue32(fig, static_cast<int32_t>(value * 1000.0f));
  }

  void FillInt(Referee::UIFigure& fig, const char* name, Referee::UIFigureOp op,
               uint8_t layer, Referee::UIColor color, uint16_t font_size,
               uint16_t width, uint16_t x, uint16_t y, int32_t value) {
    InitFigure(fig, name, op, Referee::UIFigureType::UI_TYPE_INT, layer, color,
               width, x, y);
    fig.details_a = font_size;
    SetFigureValue32(fig, value);
  }

  uint16_t GetClientID(uint16_t robot_id) {
    if (robot_id > 100) {
      return static_cast<uint16_t>(robot_id - 101 + 0x0165);
    }
    return static_cast<uint16_t>(robot_id + 0x0100);
  }

  static void SetFigureName(uint8_t (&dst)[3], const char* name) {
    dst[0] = ' ';
    dst[1] = ' ';
    dst[2] = ' ';
    if (name == nullptr) {
      return;
    }
    for (int i = 0; i < 3 && name[i] != '\0'; ++i) {
      dst[i] = static_cast<uint8_t>(name[i]);
    }
  }

  LibXR::ErrorCode SendUILayerDelete(uint16_t sender_id, uint16_t receiver_id,
                                     const UILayerDelete& payload) {
    return SendStudentCmd(CMDID::REF_STDNT_CMD_ID_UI_DEL, sender_id,
                          receiver_id, payload);
  }

  LibXR::ErrorCode SendUIFigure(uint16_t sender_id, uint16_t receiver_id,
                                const UIFigure& payload) {
    return SendStudentCmd(CMDID::REF_STDNT_CMD_ID_UI_DRAW1, sender_id,
                          receiver_id, payload);
  }

  LibXR::ErrorCode SendUIFigure2(uint16_t sender_id, uint16_t receiver_id,
                                 const UIFigure2& payload) {
    return SendStudentCmd(CMDID::REF_STDNT_CMD_ID_UI_DRAW2, sender_id,
                          receiver_id, payload);
  }

  LibXR::ErrorCode SendUIFigure5(uint16_t sender_id, uint16_t receiver_id,
                                 const UIFigure5& payload) {
    return SendStudentCmd(CMDID::REF_STDNT_CMD_ID_UI_DRAW5, sender_id,
                          receiver_id, payload);
  }

  LibXR::ErrorCode SendUIFigure7(uint16_t sender_id, uint16_t receiver_id,
                                 const UIFigure7& payload) {
    return SendStudentCmd(CMDID::REF_STDNT_CMD_ID_UI_DRAW7, sender_id,
                          receiver_id, payload);
  }

  LibXR::ErrorCode SendUICharacter(uint16_t sender_id, uint16_t receiver_id,
                                   const UICharacter& payload) {
    return SendStudentCmd(CMDID::REF_STDNT_CMD_ID_UI_STR, sender_id,
                          receiver_id, payload);
  }

  LibXR::ErrorCode SendSentryDecision(const SentryDecisionData& payload) {
    return SendStudentCmd(
        CMDID::REF_STDNT_CMD_ID_SENTRY_CMD, GetRobotID(),
        static_cast<uint16_t>(ClientID::REF_CL_REFEREE_SERVER), payload);
  }

  LibXR::ErrorCode SendRadarDecision(const RadarDecisionData& payload) {
    return SendStudentCmd(
        CMDID::REF_STDNT_CMD_ID_RADAR_CMD, GetRobotID(),
        static_cast<uint16_t>(ClientID::REF_CL_REFEREE_SERVER), payload);
  }

  LibXR::ErrorCode SendSetVideoTransChannel(uint8_t channel) {
    SetVideoTransChannel payload{};
    payload.value = channel;
    return SendFrame(CommandID::REF_CMD_ID_SET_VIDEO_TRANS_CH, payload);
  }

  LibXR::ErrorCode SendQueryVideoTransChannel() {
    return SendFrame(CommandID::REF_CMD_ID_GET_VIDEO_TRANS_CH, nullptr, 0);
  }

  LibXR::ErrorCode SendCustomDataToController(const CustomData1& payload) {
    return SendFrame(CommandID::REF_CMD_ID_CUSTOM_RECV_DATA, payload);
  }

  LibXR::ErrorCode SendCustomDataToController(const CustomData2& payload) {
    return SendFrame(CommandID::REF_CMD_ID_DATA_TO_CUSTOM_CLIENT, payload);
  }

  /**
   * @brief 线程函数
   *
   */
  static void ThreadFunc(Referee* ref) {
    while (true) {
      ref->ServiceTx();
      ref->last_parse_ = ref->FindHeader() && ref->ParseData();
      ref->Publish();
      ref->MaintainLink();
      ref->ServiceTx();
      // Yield without imposing a per-frame delay on buffered UART traffic.
      LibXR::Thread::Sleep(0);
    }
  }

  bool FindHeader() {
    // The same bounded stream buffer retains partial headers across read waits.
    for (size_t scan = 0; scan < sizeof(rx_stream_); ++scan) {
      while (rx_size_ > 0 && rx_stream_[0] != 0xa5U) DiscardRx(1);
      if (rx_size_ >= sizeof(Header)) {
        if (LibXR::CRC8::Verify(rx_stream_, sizeof(Header))) {
          LibXR::Memory::FastCopy(&pack_.header_, rx_stream_, sizeof(Header));
          const size_t total = sizeof(Header) + sizeof(uint16_t) +
                               pack_.header_.data_length + sizeof(uint16_t);
          if (total <= sizeof(rx_stream_)) return true;
        }
        DiscardRx(1);
        continue;
      }
      if (!ReadRxByte()) return false;
    }
    return false;
  }

  /**
   * @brief 解析包数据的函数
   *
   * @return true 成功解析
   * @return false
   */
  bool ParseData() {
    const uint16_t PAYLOAD_LEN = this->pack_.header_.data_length;
    const size_t BYTES_AFTER_HEADER =
        sizeof(uint16_t) + PAYLOAD_LEN + sizeof(uint16_t);

    if (BYTES_AFTER_HEADER > sizeof(this->pack_.buf_)) {
      return false;
    }

    const size_t total = sizeof(Header) + BYTES_AFTER_HEADER;
    while (rx_size_ < total) {
      if (!ReadRxByte()) return false;
    }
    if (!LibXR::CRC16::Verify(rx_stream_, total)) {
      // Preserve any intact frame embedded after a truncated/bad frame.
      DiscardRx(1);
      return false;
    }
    LibXR::Memory::FastCopy(&pack_, rx_stream_, total);
    DiscardRx(total);

    const uint16_t CMD_ID = static_cast<uint16_t>(this->pack_.buf_[0]) |
                            (static_cast<uint16_t>(this->pack_.buf_[1]) << 8);
    const uint8_t* payload = &this->pack_.buf_[2];

    const auto command_id = static_cast<CommandID>(CMD_ID);
    const int expected = ExpectedPayloadSize(command_id);
    if (expected == -2 || (expected >= 0 && PAYLOAD_LEN != expected) ||
        (expected == -1 && !ValidInteraction(payload, PAYLOAD_LEN)))
      return false;
    const auto COPY_PAYLOAD = [payload, PAYLOAD_LEN](auto& dst) -> bool {
      if (PAYLOAD_LEN != sizeof(dst)) return false;
      LibXR::Memory::FastCopy(&dst, payload, sizeof(dst));
      return true;
    };

    {
      LibXR::Mutex::LockGuard lock(tx_data_mutex_);
      switch (static_cast<CommandID>(CMD_ID)) {
        case CommandID::REF_CMD_ID_GAME_STATUS: {
          /* 0x0001, 比赛状态数据 */
          if (!COPY_PAYLOAD(this->data_.game_status)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_GAME_RESULT: {
          /* 0x0002, 比赛结果数据 */
          if (!COPY_PAYLOAD(this->data_.game_result)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_GAME_ROBOT_HP: {
          /* 0x0003, 机器人血量数据 */
          if (!COPY_PAYLOAD(this->data_.game_robot_hp)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_FIELD_EVENTS: {
          /* 0x0101, 场地事件数据 */
          if (!COPY_PAYLOAD(this->data_.field_event)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_WARNING: {
          /* 0x0104, 裁判警告数据 */
          if (!COPY_PAYLOAD(this->data_.warning)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_DART_COUNTDOWN: {
          /* 0x0105, 飞镖发射相关数据 */
          if (!COPY_PAYLOAD(this->data_.dart_countdown)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_ROBOT_STATUS: {
          /* 0x0201, 机器人性能体系数据 */
          if (!COPY_PAYLOAD(this->data_.robot_status)) {
            return false;
          }
          this->robot_status_received_time_ms_ =
              static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
          this->robot_status_received_ = true;
          break;
        }

        case CommandID::REF_CMD_ID_POWER_HEAT_DATA: {
          /* 0x0202, 实时底盘缓冲能量和射击热量 */
          if (!COPY_PAYLOAD(this->data_.power_heat)) {
            return false;
          }
          this->power_heat_received_time_ms_ =
              static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
          this->power_heat_received_ = true;
          break;
        }

        case CommandID::REF_CMD_ID_ROBOT_POS: {
          /* 0x0203, 机器人位置数据 */
          if (!COPY_PAYLOAD(this->data_.robot_pos)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_ROBOT_BUFF: {
          /* 0x0204, 机器人增益和底盘能量数据 */
          if (!COPY_PAYLOAD(this->data_.robot_buff)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_DRONE_ENERGY:
          return false;  // legacy 2026

        case CommandID::REF_CMD_ID_ROBOT_DMG: {
          /* 0x0206, 伤害状态数据 */
          if (!COPY_PAYLOAD(this->data_.robot_damage)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_LAUNCHER_DATA: {
          /* 0x0207, 实时射击数据 */
          if (!COPY_PAYLOAD(this->data_.launcher_data)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_BULLET_REMAINING: {
          /* 0x0208, 允许发弹量 */
          if (!COPY_PAYLOAD(this->data_.bullet_remain)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RFID: {
          /* 0x0209, 机器人 RFID 模块状态 */
          if (!COPY_PAYLOAD(this->data_.rfid)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_DART_CLIENT: {
          /* 0x020A, 飞镖选手端指令数据 */
          if (!COPY_PAYLOAD(this->data_.dart_client)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_ROBOT_POS_TO_SENTRY: {
          /* 0x020B, 地面机器人位置数据 -> 哨兵 */
          if (!COPY_PAYLOAD(this->data_.sentry_pos)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_MARK: {
          /* 0x020C, 雷达标记进度数据 */
          if (!COPY_PAYLOAD(this->data_.radar_mark_progress)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_SENTRY_DECISION: {
          /* 0x020D, 哨兵自主决策相关信息同步 */
          if (PAYLOAD_LEN != sizeof(SentryInfo) ||
              !COPY_PAYLOAD(this->data_.sentry_decision)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_DECISION: {
          /* 0x020E, 雷达自主决策相关信息同步 */
          if (!COPY_PAYLOAD(this->data_.radar_decision)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_INTER_STUDENT: {
          /* 0x0301, 机器人交互数据 */
          if (PAYLOAD_LEN < 6) {
            return false;
          }
          LibXR::Memory::FastSet(
              this->data_.robot_ineraction_data.user_data, 0,
              sizeof(this->data_.robot_ineraction_data.user_data));
          LibXR::Memory::FastCopy(&this->data_.robot_ineraction_data, payload,
                                  6);
          const size_t USER_DATA_LEN = std::min<size_t>(
              PAYLOAD_LEN - 6,
              sizeof(this->data_.robot_ineraction_data.user_data));
          if (USER_DATA_LEN > 0) {
            LibXR::Memory::FastCopy(this->data_.robot_ineraction_data.user_data,
                                    payload + 6, USER_DATA_LEN);
          }
          break;
        }

        case CommandID::REF_CMD_ID_INTER_STUDENT_CUSTOM: {
          /* 0x0302, 自定义控制器和机器人（图传） */
          if (!COPY_PAYLOAD(this->data_.custom_controller)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_CLIENT_MAP: {
          /* 0x0303, 选手端小地图交互数据 */
          if (!COPY_PAYLOAD(this->data_.client_map)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_KEYBOARD_MOUSE: {
          /* 0x0304, 键鼠遥控数据 */
          if (!COPY_PAYLOAD(this->data_.keyboard_mouse)) {
            return false;
          }
          FeedVideoLinkRemoteToCMD(this->data_.keyboard_mouse);
          break;
        }

        case CommandID::REF_CMD_ID_MAP_ROBOT_DATA: {
          /* 0x0305, 小地图接收雷达数据 */
          if (!COPY_PAYLOAD(this->data_.map_robot_data)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_CUSTOM_KEYBOARD_MOUSE: {
          /* 0x0306, 自定义控制器交互（键鼠模拟） */
          if (!COPY_PAYLOAD(this->data_.custom_key_mouse_data)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_SENTRY_POS_DATA: {
          /* 0x0307, 小地图接收路径数据 */
          if (!COPY_PAYLOAD(this->data_.sentry_postion)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_ROBOT_POS_DATA: {
          /* 0x0308, 选手端小地图接受机器人消息 */
          if (!COPY_PAYLOAD(this->data_.robot_position)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_CUSTOM_RECV_DATA: {
          /* 0x0309, 自定义控制器收包 */
          if (!COPY_PAYLOAD(this->data_.custom_data1)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_DATA_TO_CUSTOM_CLIENT: {
          /* 0x0310, 自定义客户端数据 */
          if (!COPY_PAYLOAD(this->data_.custom_data2)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_SET_VIDEO_TRANS_CH: {
          /* 0x0F01, 设置图传出图信道 (应答) */
          if (!COPY_PAYLOAD(this->data_.set_vid_trans_ch)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_GET_VIDEO_TRANS_CH: {
          /* 0x0F02, 查询当前出图信道 (应答) */
          if (!COPY_PAYLOAD(this->data_.get_vid_trans_ch)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_ENEMY_POS: {
          /* 0x0A01, 对方机器人的位置坐标 */
          if (!COPY_PAYLOAD(this->data_.radar_enemy_robot_pos)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_ENEMY_HP: {
          /* 0x0A02, 对方机器人的血量信息 */
          if (!COPY_PAYLOAD(this->data_.radar_enemy_robot_hp)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_ENEMY_BULLET: {
          /* 0x0A03, 对方剩余发弹量 */
          if (!COPY_PAYLOAD(this->data_.radar_enemy_bullet)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_ENEMY_INFO: {
          /* 0x0A04, 对方宏观状态信息 */
          if (!COPY_PAYLOAD(this->data_.radar_enemy_state)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_ENEMY_BUFF: {
          /* 0x0A05, 对方增益效果 */
          if (!COPY_PAYLOAD(this->data_.radar_robot_buff)) {
            return false;
          }
          break;
        }

        case CommandID::REF_CMD_ID_RADAR_ENEMY_KEY: {
          /* 0x0A06, 对方干扰波密钥 */
          if (!COPY_PAYLOAD(this->data_.radar_enemy_key)) {
            return false;
          }
          break;
        }

        default: {
          /* 未知命令 */
          return false;
        }
      }
      this->data_.status = Status::RUNNING;
    }
    this->last_wake_up_ = LibXR::Timebase::GetMilliseconds();
    UpdateRefereeSource(command_id);
    if (command_id == CommandID::REF_CMD_ID_SENTRY_DECISION)
      UpdateDecisionFeedback();
    if (command_id == CommandID::REF_CMD_ID_GAME_STATUS) TrackMatchTransition();
    return true;
  }

  /**
   * @brief 缓存图传遥控数据，由 CMD 所属任务提交
   *
   * @param rc
   */
  void FeedVideoLinkRemoteToCMD(const KeyboardMouse& rc) {
    /* 接收线程仅缓存，控制任务负责超时与提交。 */
    LibXR::Mutex::LockGuard lock(video_link_mutex_);

    constexpr float MOUSE_SCALER = 1000.0f / 32768.0f;
    CMD::Data cmd_data{};

    const uint16_t KEY = rc.keyboard_value;
    if (KEY & static_cast<uint16_t>(KeyBoard::KEY_A)) {
      cmd_data.chassis.vx -= 0.5f;
    }
    if (KEY & static_cast<uint16_t>(KeyBoard::KEY_D)) {
      cmd_data.chassis.vx += 0.5f;
    }
    if (KEY & static_cast<uint16_t>(KeyBoard::KEY_S)) {
      cmd_data.chassis.vy -= 0.5f;
    }
    if (KEY & static_cast<uint16_t>(KeyBoard::KEY_W)) {
      cmd_data.chassis.vy += 0.5f;
    }
    if (KEY & static_cast<uint16_t>(KeyBoard::KEY_SHIFT)) {
      cmd_data.chassis.vx *= 2.0f;
      cmd_data.chassis.vy *= 2.0f;
    }
    cmd_data.chassis.wz = 0.0f;

    cmd_data.gimbal.pit = -static_cast<float>(rc.mouse_y) * MOUSE_SCALER;
    cmd_data.gimbal.yaw = -static_cast<float>(rc.mouse_x) * MOUSE_SCALER;
    cmd_data.gimbal.rol = 0.0f;

    cmd_data.launcher.isfire = (rc.button_l != 0);
    cmd_data.chassis_online = true;
    cmd_data.gimbal_online = true;
    cmd_data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_RC;
    video_link_pending_ = cmd_data;
    video_link_dirty_ = true;
    video_link_stop_pending_ |= !cmd_data.launcher.isfire;

    const auto now = LibXR::Timebase::GetMilliseconds();
    video_link_gap_pending_ |=
        video_link_remote_online_ &&
        (now - video_link_remote_last_time_).ToMillisecond() >
            VIDEO_LINK_REMOTE_TIMEOUT_MS;
    video_link_remote_last_time_ = now;
    video_link_remote_online_ = true;
  }

  /**
   * @brief 广播数据的函数
   *
   */
  void Publish() {
    if (!this->last_parse_) {
      return;
    }
    this->cp_.rs = this->data_.robot_status;
    this->cp_.power_buffer = this->data_.power_heat.chassis_pwr_buff;
    this->cp_.robot_status_received_time_ms =
        this->robot_status_received_time_ms_;
    this->cp_.robot_status_received = this->robot_status_received_;
    this->cp_.power_heat_received_time_ms = this->power_heat_received_time_ms_;
    this->cp_.power_heat_received = this->power_heat_received_;
    this->chassispack_topic_.Publish(this->cp_);
    this->lp_.rs = this->data_.robot_status;
    this->lp_.rb = this->data_.robot_buff;
    this->lp_.ld = this->data_.launcher_data;
    this->lp_.dc = this->data_.dart_client;
    this->lp_.ph = this->data_.power_heat;
    this->launcherpack_topic_.Publish(this->lp_);
    this->robot_game_referee_pack_.robot_status = this->data_.robot_status;
    this->robot_game_referee_pack_.game_status = this->data_.game_status;
    this->robot_game_referee_pack_.robot_hp = this->data_.game_robot_hp;
    this->robot_game_referee_pack_.field_event = this->data_.field_event;
    this->robot_game_referee_pack_.power_heat = this->data_.power_heat;
    this->robot_game_referee_pack_.robot_pos = this->data_.robot_pos;
    this->robot_game_referee_pack_.robot_buff = this->data_.robot_buff;
    this->robot_game_referee_pack_.robot_damage = this->data_.robot_damage;
    this->robot_game_referee_pack_.bullet_remain = this->data_.bullet_remain;
    this->robot_game_referee_pack_.rfid = this->data_.rfid;
    this->robot_game_referee_pack_.sentry_info = this->data_.sentry_decision;
    this->robot_game_referee_pack_.sentry_pos = this->data_.sentry_pos;
    this->robot_game_referee_pack_.source_command_id = source_command_id_;
    this->robot_game_referee_pack_.source_valid_mask = source_valid_mask_;
    this->robot_game_referee_pack_.source_fresh_mask = FreshSourceMask(
        source_valid_mask_, source_received_ms_,
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()));
    this->robot_game_referee_pack_.referee_online = referee_online_;
    this->robot_game_referee_pack_.robot_hp_received_time_ms =
        robot_hp_received_time_ms_;
    this->robot_game_referee_pack_.sentry_pos_received_time_ms =
        sentry_pos_received_time_ms_;
    this->robot_game_referee_topic_.Publish(this->robot_game_referee_pack_);
    UpdateRadarPack();
    // this->radar_pack_topic_.Publish(this->radar_pack_);
  }

  void OnMonitor() {
    if (cmd_ != nullptr) ConsumeVideoLink();
  }

 private:
  struct PendingFrame {
    CommandID command{};
    uint16_t length = 0;
    uint8_t payload[503]{};
    bool decision = false;
  };

  static bool ValidRobotID(uint16_t id) {
    return (id >= 1U && id <= 11U) || (id >= 101U && id <= 111U);
  }

  void TrackMatchTransition() {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    const uint8_t stage = data_.game_status.game_progress;
    // Only an observed end-of-match followed by preparation opens a reset.
    if (stage == 5U) match_finished_ = true;
    if (match_finished_ && stage >= 1U && stage <= 3U) {
      new_match_pending_ = true;
      match_finished_ = false;
    }
  }

  void UpdateDecisionFeedback() {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    const uint32_t info = data_.sentry_decision.info_1;
    const uint16_t bullets = info & 0x7ffU;
    const uint8_t remote_bullet = (info >> 11U) & 0x0fU;
    const uint8_t remote_hp = (info >> 15U) & 0x0fU;
    if (!decision_baseline_ || (new_match_pending_ && bullets == 0U &&
                                remote_bullet == 0U && remote_hp == 0U)) {
      data_.sentry_dec_data.buy_bullet_num = bullets;
      confirmed_remote_bullet_ = remote_bullet;
      confirmed_remote_hp_ = remote_hp;
      pending_remote_bullet_ = pending_remote_hp_ = 0;
      data_.sentry_dec_data.remote_buy_bullet_times = remote_bullet;
      data_.sentry_dec_data.romote_buy_hp_times = remote_hp;
      decision_baseline_ = true;
      new_match_pending_ = false;
    } else {
      // A decrease is not a new-match signal. Acknowledge only the outstanding
      // +1 command, never consume more than one queued request per feedback.
      if (pending_remote_bullet_ != 0U &&
          remote_bullet == confirmed_remote_bullet_ + 1U) {
        confirmed_remote_bullet_ = remote_bullet;
        --pending_remote_bullet_;
      }
      if (pending_remote_hp_ != 0U && remote_hp == confirmed_remote_hp_ + 1U) {
        confirmed_remote_hp_ = remote_hp;
        --pending_remote_hp_;
      }
      data_.sentry_dec_data.remote_buy_bullet_times =
          confirmed_remote_bullet_ + (pending_remote_bullet_ != 0U ? 1U : 0U);
      data_.sentry_dec_data.romote_buy_hp_times =
          confirmed_remote_hp_ + (pending_remote_hp_ != 0U ? 1U : 0U);
    }
    decision_pending_ = decision_pending_ || pending_remote_bullet_ != 0 ||
                        pending_remote_hp_ != 0;
  }

  bool DecisionNeeded(uint32_t now) {
    if (!referee_online_ || !ValidRobotID(data_.robot_status.robot_id))
      return false;
    const bool fresh_status =
        robot_status_received_ && now - robot_status_received_time_ms_ <= 1000U;
    auto& request = data_.sentry_dec_data;
    if (!fresh_status || data_.robot_status.remain_hp != 0U)
      request.confirm_resurrection = 0U;
    return decision_pending_ ||
           (fresh_status && request.confirm_resurrection) ||
           (decision_baseline_ &&
            (pending_remote_bullet_ != 0 || pending_remote_hp_ != 0 ||
             request.buy_bullet_num > (data_.sentry_decision.info_1 & 0x7ffU) ||
             request.current_state != data_.sentry_decision.CurrentStance()));
  }

  bool TxDue(CommandID command, uint32_t now) const {
    if (command == CommandID::REF_CMD_ID_INTER_STUDENT)
      return !interaction_attempted_ ||
             now - last_interaction_attempt_ms_ >= 34U;
    if (command == CommandID::REF_CMD_ID_MAP_ROBOT_DATA)
      return !map_attempted_ || now - last_map_attempt_ms_ >= 200U;
    if (command == CommandID::REF_CMD_ID_DATA_TO_CUSTOM_CLIENT)
      return !custom_attempted_ || now - last_custom_attempt_ms_ >= 20U;
    return true;
  }

  void ServiceTx() {
    const uint32_t now =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (tx_active_) {
      if (!tx_finished_.load(std::memory_order_acquire)) return;
      tx_active_ = false;
      if (tx_completion_.load(std::memory_order_relaxed) ==
          LibXR::ErrorCode::OK) {
        tx_retry_ = false;
      } else if (active_frame_.decision) {
        // Preserve the merged request while allowing queued traffic its turn.
        LibXR::Mutex::LockGuard lock(tx_data_mutex_);
        decision_pending_ = true;
        tx_retry_ = false;
      } else {
        // A failed ordinary request keeps its data, but need not monopolize
        // the writer while another command is due. Hold it only if full.
        tx_retry_ = tx_queue_.Push(active_frame_) != LibXR::ErrorCode::OK;
      }
    }
    if (!tx_retry_) {
      bool decision = false;
      {
        LibXR::Mutex::LockGuard lock(tx_data_mutex_);
        decision = DecisionNeeded(now) &&
                   TxDue(CommandID::REF_CMD_ID_INTER_STUDENT, now) &&
                   (!last_was_decision_ || tx_queue_.Size() == 0U);
        if (decision) {
          active_frame_ = {};
          active_frame_.command = CommandID::REF_CMD_ID_INTER_STUDENT;
          active_frame_.length = 6U + sizeof(SentryDecisionData);
          active_frame_.decision = true;
          const uint16_t words[3] = {0x0120U, data_.robot_status.robot_id,
                                     0x8080U};
          LibXR::Memory::FastCopy(active_frame_.payload, words, sizeof(words));
          LibXR::Memory::FastCopy(active_frame_.payload + 6U,
                                  &data_.sentry_dec_data,
                                  sizeof(SentryDecisionData));
          decision_pending_ = false;
        }
      }
      if (!decision) {
        bool found = false;
        const size_t count = tx_queue_.Size();
        for (size_t i = 0; i < count; ++i) {
          if (tx_queue_.Pop(active_frame_) != LibXR::ErrorCode::OK) break;
          if (TxDue(active_frame_.command, now)) {
            found = true;
            break;
          }
          // Consumer frees one slot before requeue; hold a retry copy if a
          // concurrent producer claimed that slot, rather than dropping data.
          if (tx_queue_.Push(active_frame_) != LibXR::ErrorCode::OK) {
            tx_retry_ = true;
            return;
          }
        }
        if (!found) return;
      }
    }
    if (!TxDue(active_frame_.command, now)) return;
    if (active_frame_.command == CommandID::REF_CMD_ID_INTER_STUDENT) {
      LibXR::Mutex::LockGuard lock(tx_data_mutex_);
      const uint16_t sender =
          active_frame_.payload[2] | (active_frame_.payload[3] << 8U);
      if (!ValidRobotID(data_.robot_status.robot_id) ||
          (!active_frame_.decision && sender != data_.robot_status.robot_id)) {
        tx_completion_.store(LibXR::ErrorCode::STATE_ERR);
        tx_retry_ = false;
        return;
      }
      if (active_frame_.decision && !referee_online_) {
        tx_retry_ = false;
        decision_pending_ = true;
        return;
      }
      if (active_frame_.decision) {
        // Refresh both identity and merged intent immediately before writing.
        // A stale death report must not survive a failed UART attempt.
        if (!robot_status_received_ ||
            now - robot_status_received_time_ms_ > 1000U ||
            data_.robot_status.remain_hp != 0U)
          data_.sentry_dec_data.confirm_resurrection = 0U;
        active_frame_.payload[2] = data_.robot_status.robot_id;
        active_frame_.payload[3] = 0;
        LibXR::Memory::FastCopy(active_frame_.payload + 6U,
                                &data_.sentry_dec_data,
                                sizeof(SentryDecisionData));
      }
    }
    const auto id = active_frame_.command;
    if (id == CommandID::REF_CMD_ID_INTER_STUDENT) {
      interaction_attempted_ = true;
      last_interaction_attempt_ms_ = now;
    } else if (id == CommandID::REF_CMD_ID_MAP_ROBOT_DATA) {
      map_attempted_ = true;
      last_map_attempt_ms_ = now;
    } else if (id == CommandID::REF_CMD_ID_DATA_TO_CUSTOM_CLIENT) {
      custom_attempted_ = true;
      last_custom_attempt_ms_ = now;
    }
    Header header{};
    header.data_length = active_frame_.length;
    header.seq = tx_seq_++;
    header.crc8 = LibXR::CRC8::Calculate(&header, sizeof(Header) - 1U);
    LibXR::Memory::FastCopy(tx_buf_, &header, sizeof(header));
    const uint16_t cmd = static_cast<uint16_t>(id);
    tx_buf_[5] = cmd & 0xffU;
    tx_buf_[6] = cmd >> 8U;
    LibXR::Memory::FastCopy(tx_buf_ + 7U, active_frame_.payload,
                            active_frame_.length);
    const size_t length = 7U + active_frame_.length;
    const uint16_t crc = LibXR::CRC16::Calculate(tx_buf_, length);
    tx_buf_[length] = crc & 0xffU;
    tx_buf_[length + 1U] = crc >> 8U;
    last_was_decision_ = active_frame_.decision;
    tx_finished_.store(false, std::memory_order_release);
    tx_active_ = true;
    tx_retry_ = false;
    const auto result = uart_->Write({tx_buf_, length + 2U}, tx_op_);
    if (result != LibXR::ErrorCode::OK) {
      tx_completion_.store(result, std::memory_order_relaxed);
      tx_finished_.store(true, std::memory_order_release);
    }
  }

  void DiscardRx(size_t count) {
    rx_size_ -= count;
    if (rx_size_ > 0) std::memmove(rx_stream_, rx_stream_ + count, rx_size_);
  }

  bool ReadRxByte() {
    ServiceTx();
    MaintainLink();
    const auto now = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (rx_size_ != 0 && now - rx_last_byte_ms_ > 50U) {
      // Restart header search as well as dropping the stale bytes. ParseData
      // must not keep waiting for the length advertised by that old header.
      rx_size_ = 0;
      return false;
    }
    uint8_t value = 0;
    if (uart_->Read({&value, 1}, op_) != LibXR::ErrorCode::OK) return false;
    if (rx_size_ >= sizeof(rx_stream_)) DiscardRx(1);
    rx_stream_[rx_size_++] = value;
    rx_last_byte_ms_ =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    return true;
  }

  void MaintainLink() {
    const auto now = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (referee_online_ && now - static_cast<uint32_t>(last_wake_up_) > 1000U) {
      {
        LibXR::Mutex::LockGuard lock(tx_data_mutex_);
        data_.status = Status::OFFLINE;
        data_.sentry_dec_data.confirm_resurrection = 0U;
      }
      robot_status_received_ = power_heat_received_ = false;
      PublishRefereeStatusIfChanged(false);
      return;
    }
    const uint16_t fresh =
        FreshSourceMask(source_valid_mask_, source_received_ms_, now);
    if (fresh != robot_game_referee_pack_.source_fresh_mask) {
      robot_game_referee_pack_.source_fresh_mask = fresh;
      robot_game_referee_pack_.source_command_id = 0;
      robot_game_referee_topic_.Publish(robot_game_referee_pack_);
    }
  }

  static constexpr uint32_t VIDEO_LINK_REMOTE_TIMEOUT_MS = 100;

  void UpdateRefereeSource(CommandID command_id) {
    const uint16_t source_mask = SourceMaskForCommand(command_id);
    if (source_mask == SOURCE_ROBOT_HP)
      robot_hp_received_time_ms_ =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (source_mask == SOURCE_SENTRY_POS)
      sentry_pos_received_time_ms_ =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    referee_online_ = true;
    source_command_id_ = static_cast<uint16_t>(command_id);
    source_valid_mask_ |= source_mask;
    if (source_mask != 0U) {
      for (size_t index = 0; index < source_received_ms_.size(); ++index) {
        if (source_mask == (1U << index)) {
          source_received_ms_[index] =
              static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
          break;
        }
      }
    }
  }

  void PublishRefereeStatusIfChanged(bool online) {
    if (referee_online_ == online) {
      return;
    }

    referee_online_ = online;
    source_command_id_ = 0U;
    if (!online) {
      source_valid_mask_ = 0U;
      for (auto& received_ms : source_received_ms_) received_ms = 0U;
      sentry_pos_received_time_ms_ = 0U;
      robot_hp_received_time_ms_ = 0U;
    }
    robot_game_referee_pack_.source_command_id = source_command_id_;
    robot_game_referee_pack_.source_valid_mask = source_valid_mask_;
    robot_game_referee_pack_.source_fresh_mask = 0U;
    robot_game_referee_pack_.referee_online = referee_online_;
    robot_game_referee_topic_.Publish(robot_game_referee_pack_);
  }

  void ConsumeVideoLink() {
    CMD::Data data{};
    bool publish = false;
    {
      LibXR::Mutex::LockGuard lock(video_link_mutex_);
      if (video_link_gap_pending_) {
        video_link_gap_pending_ = false;
        data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_RC;
        // Keep the fresh buffered frame for the next owner cycle.
        publish = true;
      } else {
        const auto now = LibXR::Timebase::GetMilliseconds();
        const bool fresh =
            video_link_remote_online_ &&
            (now - video_link_remote_last_time_).ToMillisecond() <=
                VIDEO_LINK_REMOTE_TIMEOUT_MS;
        if (fresh) data = video_link_pending_;
        if (video_link_stop_pending_) data.launcher.isfire = false;
        publish = video_link_dirty_ || video_link_stop_pending_ ||
                  (!fresh && video_link_remote_online_);
        video_link_dirty_ = fresh && video_link_stop_pending_ &&
                            video_link_pending_.launcher.isfire;
        video_link_stop_pending_ = false;
        if (!fresh) video_link_remote_online_ = false;
      }
    }
    if (publish) cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_REFEREE, data);
  }

  void UpdateRadarPack() {
    LibXR::Mutex::LockGuard lock(tx_data_mutex_);
    this->radar_pack_.rf = this->data_.sentry_pos;
    this->radar_pack_.x = this->data_.robot_pos.x;
    this->radar_pack_.y = this->data_.robot_pos.y;
  }

  /* 数据读取 */
  bool referee_online_ = false;
  bool last_parse_ = false; /* 上一次解包是否成功 */
  bool video_link_ = false;
  bool decision_baseline_ = false;
  bool new_match_pending_ = false;
  bool match_finished_ = false;
  uint8_t confirmed_remote_bullet_ = 0, confirmed_remote_hp_ = 0;
  uint8_t pending_remote_bullet_ = 0, pending_remote_hp_ = 0;
  bool decision_pending_ = false;
  bool interaction_attempted_ = false, map_attempted_ = false,
       custom_attempted_ = false;
  uint32_t last_interaction_attempt_ms_ = 0, last_map_attempt_ms_ = 0,
           last_custom_attempt_ms_ = 0;
  bool tx_active_ = false, tx_retry_ = false, last_was_decision_ = false;
  std::atomic<bool> tx_finished_{false};
  std::atomic<LibXR::ErrorCode> tx_completion_{LibXR::ErrorCode::OK};
  std::atomic<uint32_t> tx_admission_errors_{0};
  Data data_{}; /* 数据包本体 */

  ChassisPack cp_;  /* 发给底盘的数据包缓冲 */
  LauncherPack lp_; /* 发给发射的数据包缓冲 */
  RobotGameRefereePack robot_game_referee_pack_; /* 裁判系统摘要包缓冲 */
  RadarPack radar_pack_;

  uint32_t robot_status_received_time_ms_ = 0U;
  uint32_t power_heat_received_time_ms_ = 0U;
  bool robot_status_received_ = false;
  bool power_heat_received_ = false;
  uint16_t source_command_id_ = 0U;
  SourceTimes source_received_ms_{};
  uint16_t source_valid_mask_ = 0U;
  uint32_t sentry_pos_received_time_ms_ = 0U;
  uint32_t robot_hp_received_time_ms_ = 0U;
  LibXR::MillisecondTimestamp
      video_link_remote_last_time_{}; /* 图传键鼠最近一次写入 CMD 的时间 */
  bool video_link_remote_online_ = false; /* 图传键鼠在线锁存，超时后清零 */
  CMD::Data video_link_pending_{};
  bool video_link_dirty_ = false;
  bool video_link_gap_pending_ = false;
  bool video_link_stop_pending_ = false; /* 保护图传键鼠在线状态 */

  /* 协议,数据流 */
  struct [[gnu::packed]] {
    Header header_;
    uint8_t buf_[507]; /* 完整帧容量 512 */
  } pack_;
  uint8_t tx_buf_[512]{};
  uint8_t rx_stream_[512]{};
  size_t rx_size_ = 0;
  uint32_t rx_last_byte_ms_ = 0;
  uint8_t tx_seq_ = 0;

  PendingFrame active_frame_{};
  LibXR::MPMCQueue<PendingFrame> tx_queue_{8};
  LibXR::UART* uart_;
  LibXR::Semaphore sem_;
  LibXR::ReadOperation op_;
  LibXR::Semaphore sem_tx_;
  LibXR::WriteOperation::Callback tx_callback_;
  LibXR::WriteOperation tx_op_;
  mutable LibXR::Mutex tx_data_mutex_;
  LibXR::Mutex video_link_mutex_;
  CMD* cmd_;
  LibXR::Topic chassispack_topic_;
  LibXR::Topic launcherpack_topic_;
  LibXR::Topic robot_game_referee_topic_;
  LibXR::Topic radar_pack_topic_;
  /* 线程相关 */
  LibXR::MillisecondTimestamp last_wake_up_{}; /* ms */
  LibXR::Thread thread_;
};
