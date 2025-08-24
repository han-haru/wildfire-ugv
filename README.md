# wildfire-ugv

Gazebo sim, RViz2, ros2_control 사용 
사용방법
1. 워크스페이스 소스
2. ros2 launch fire_bringup gz_spawn.launch.py # Gazebo sim 및 RViz2 환경 실행
3. ros2 launch fire_bringup controllers.launch.py # Controller 업데이트; 이걸 해야 제대로 된 모델 셋팅됨
4. ros2 run fire_tracks cmdvel_ramp_to_tracks.py # 민수가 말한 0.5초 가속 코드
5. ros2 run fire_tracks cmd_vel.py # cmd_vel 명령 쉽게 내릴 수 있는 코드
