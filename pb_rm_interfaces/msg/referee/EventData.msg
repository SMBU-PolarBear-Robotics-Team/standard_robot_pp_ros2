# msg for referee system (V1.6.4 0x0101)

# Constants for the occupation and activation states
uint8 UNOCCUPIED = 0                # Not occupied or not activated
uint8 OCCUPIED_FRIEND = 1           # Occupied or activated by friendly side
uint8 OCCUPIED_ENEMY = 2            # Occupied or activated by enemy side
uint8 OCCUPIED_BOTH = 3             # Occupied or activated by both sides

uint8 supply_station_front          # 己方补给站前补血点的占领状态，1 为已占领
uint8 supply_station_internal       # 己方补给站内部补血点的占领状态，1 为已占领
uint8 supply_zone                   # 己方补给区的占领状态，1 为已占领
uint8 center_gain_zone              # 中心增益点的占领情况，
                                    # 0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领（仅 RMUL 适用）

uint8 small_energy                  # 己方小能量机关的激活状态，1 为已激活
uint8 big_energy                    # 己方大能量机关的激活状态，1 为已激活

uint8 circular_highland             # 己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
uint8 trapezoidal_highland_3        # 己方 3 号梯形高地的占领状态，1 为被己方占领，2 为被对方占领
uint8 trapezoidal_highland_4        # 己方 4 号梯形高地的占领状态，1 为被己方占领，2 为被对方占领

uint8 base_virtual_shield_remaining # 己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
