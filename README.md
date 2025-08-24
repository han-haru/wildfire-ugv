# wildfire-ugv

Gazebo sim, RViz2, ros2_control 사용
### 패키지 소개
fire_description
로봇 자체 정의 (URDF/Xacro, meshes, materials, rviz config)
→ 즉 “로봇이 어떤 모습이냐” 만 담당.

urdf/

meshes/

materials/

(optional) rviz/

fire_bringup
실행/환경/시뮬레이션 설정 (launch, controller.yaml, world, sensor plugin config)
→ 즉 “로봇을 어떻게 띄우고 구동하냐” 담당.

launch/

config/ (ros2_control, sensors, nav2 params…)

worlds/ (실험 환경 sdf, world 파일)

## 사용방법
1. 워크스페이스 소스
2. ros2 launch fire_bringup gz_spawn.launch.py # Gazebo sim 및 RViz2 환경 실행
3. ros2 launch fire_bringup controllers.launch.py # Controller 업데이트; 이걸 해야 제대로 된 모델 셋팅됨
4. ros2 run fire_tracks cmdvel_ramp_to_tracks.py # 민수가 말한 0.5초 가속 코드
5. ros2 run fire_tracks cmd_vel.py # cmd_vel 명령 쉽게 내릴 수 있는 코드
