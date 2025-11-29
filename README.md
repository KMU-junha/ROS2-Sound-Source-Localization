# 🔊 ROS2 Sound Source Localization for Mobile Robot
> **RGB-D SLAM 기반 정찰 로봇을 위한 TDOA 소리 위치 추정 및 시공간 클러스터링 시스템**

### 📋 Project Info
* **행사:** 2025-2 국민대학교 전자공학 설계대회 (UROP II 연구 프로젝트)
* **기간:** 2025.09 ~ 2025.12
* **역할:** **Sound Localization System Lead** (소리 위치 추정 알고리즘 및 ROS2 노드 개발)
* **Teamwork:** 3D SLAM (팀원 담당) + Sound Localization (본인 담당) = **Sensor Fusion Reconnaissance Robot**

## 1. Overview (개요)
본 프로젝트는 정찰 로봇이 시각 정보(Camera/LiDAR)뿐만 아니라 **청각 정보(Sound)**를 활용하여 보이지 않는 곳의 타겟 위치를 추정할 수 있도록 하는 **Sound Localization System**입니다.
이동하는 로봇(TurtleBot3)과 관제 PC에 설치된 두 개의 마이크 어레이를 연동하여, **동적인 환경에서도 정확한 소리 발생 지점(x, y)을 삼각측량** 합니다.

## 2. Key Features & Algorithm (핵심 기술)

### 2-1. TDOA Based DOA Estimation (도래각 추정)
* **Hardware:** ReSpeaker Mic Array v2.0 (x2)
* **Algorithm:** GCC-PHAT 기반의 TDOA(Time Difference of Arrival) 알고리즘을 활용하여 소리의 도래각(DOA) 추출.
* **Implementation:** `doa_publisher.py` 노드가 USB 인터페이스로 마이크 데이터를 수신하고, 이를 ROS2 Topic(`Float32`, `PoseStamped`)으로 변환하여 실시간 발행.

### 2-2. Dynamic Triangulation with TF (이동 로봇 삼각측량) 

[Image of triangulation geometry]

* **Challenge:** 로봇이 움직이면 마이크의 위치와 각도가 계속 변하기 때문에, 단순 삼각측량으로는 좌표 계산이 불가능함.
* **Solution:**
    * **TF2 (Transform Library):** ROS2의 `tf2_ros`를 활용하여 로봇의 로컬 좌표계(Base_link) 기준의 DOA 각도를 지도(Map) 기준의 **글로벌 벡터**로 실시간 변환.
    * **Time Synchronization:** `message_filters.ApproximateTimeSynchronizer`를 사용하여 서로 다른 주기의 센서 데이터(로봇 위치, 마이크 각도)를 동기화(Sync) 처리.

### 2-3. Spatio-Temporal Clustering (시공간 클러스터링)
* **Problem:** 실내 환경의 반사음(Reverberation)과 잡음으로 인해 추정 좌표가 튀는 현상(Outlier) 발생.
* **Algorithm:**
    * **Sliding Window:** 최근 N개의 추정 좌표를 큐(Queue)에 저장.
    * **Std Filter:** 저장된 좌표들의 표준편차(Standard Deviation)가 임계값(`CLUSTER_STD_LIMIT`) 이내일 때만 유효한 타겟으로 인정.
    * **Stagnation Detection:** 센서 오류로 인해 값이 굳는(Frozen) 현상을 감지하고 필터링하는 로직 구현.

## 3. System Architecture (시스템 구조)
```mermaid
graph TD
    subgraph Robot [TurtleBot3 (Jetson Orin)]
        Mic1[ReSpeaker Mic] -->|USB| DOA_Node1[doa_publisher.py]
        DOA_Node1 -->|Topic: /mic_angle| ROS2_Network
    end

    subgraph Control_Center [Laptop]
        Mic2[ReSpeaker Mic] -->|USB| DOA_Node2[doa_publisher.py]
        DOA_Node2 -->|Topic: /mic_angle_2| ROS2_Network
        
        ROS2_Network -->|Sync| Triangulator[triangulator_tf.py]
        TF_Tree[TF / SLAM Odometry] -->|Transform| Triangulator
        
        Triangulator -->|Clustering| Estimated_Point[Target Marker]
        Estimated_Point --> RViz[RViz Visualization]
    end
