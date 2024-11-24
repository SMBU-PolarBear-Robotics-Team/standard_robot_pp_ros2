# msg for RFID detection system (V1.6.4 0x0209)

# const for RFID status
bool NOT_DETECTED = 0                       # RFID card not detected
bool DETECTED = 1                           # RFID card detected

bool base_gain_point                        # 己方基地增益点
bool circular_highland_gain_point           # 己方环形高地增益点
bool enemy_circular_highland_gain_point     # 对方环形高地增益点
bool friendly_r3_b3_gain_point              # 己方 R3/B3 梯形高地增益点
bool enemy_r3_b3_gain_point                 # 对方 R3/B3 梯形高地增益点
bool friendly_r4_b4_gain_point              # 己方 R4/B4 梯形高地增益点
bool enemy_r4_b4_gain_point                 # 对方 R4/B4 梯形高地增益点
bool energy_mechanism_gain_point            # 己方能量机关激活点
bool friendly_fly_ramp_front_gain_point     # 己方飞坡增益点（靠近己方一侧飞坡前）
bool friendly_fly_ramp_back_gain_point      # 己方飞坡增益点（靠近己方一侧飞坡后）
bool enemy_fly_ramp_front_gain_point        # 对方飞坡增益点（靠近对方一侧飞坡前）
bool enemy_fly_ramp_back_gain_point         # 对方飞坡增益点（靠近对方一侧飞坡后）
bool friendly_outpost_gain_point            # 己方前哨站增益点
bool friendly_healing_point                 # 己方补血点（检测到任一均视为激活）
bool friendly_sentry_patrol_area            # 己方哨兵巡逻区
bool enemy_sentry_patrol_area               # 对方哨兵巡逻区
bool friendly_big_resource_island           # 己方大资源岛增益点
bool enemy_big_resource_island              # 对方大资源岛增益点
bool friendly_exchange_area                 # 己方兑换区
bool center_gain_point                      # 中心增益点 RFID 卡状态（仅 RMUL 适用），1 为已检测到
