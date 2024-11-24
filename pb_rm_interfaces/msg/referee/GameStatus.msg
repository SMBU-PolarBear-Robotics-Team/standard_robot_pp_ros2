# msg for referee system (V1.6.4 0x0001)
# constants for game progress
uint8 NOT_START = 0      # game not start
uint8 PREPARATION = 1    # preparation stage (free control for robot, disable referee system)
uint8 SELF_CHECKING = 2  # referee system self-checking stage (unable to control robot, and initialize all referee system data).
uint8 COUNT_DOWN = 3     # 5 seconds count down stage (unable to control robot, enable referee system)
uint8 RUNNING = 4        # running stage (free control for robot, enable referee system)
uint8 GAME_OVER = 5      # game over stage (unable to control robot, disable referee system)

uint8 game_progress 0
int32 stage_remain_time 0           # seconds