## 제2회 미래형자동차 자율주행 SW 경진대회  
---  
### 프로젝트 소개
---  
유아용 전동차를 자율주행차로 개조하여 다음 목표를 이루도록 설계한다. 설계를 바탕으로 제작한
차량이 트랙을 2바퀴 도는 '트랙주행'과 장애물 회피, 횡단보도, 주차 미션이 포함된 '미션주행'을 성공적으로 수행하고자 한다.  

1) 주최 측에서 지정한 센서를 차량에 설치하여 차량 주행에 필요한 인식 값을 얻도록 센서 위치를 결정한다.
2) 센서 인식 값을 사람의 개입 없이 차량 스스로 위치를 인지하고 제어하는 차량 시스템을 설계한다.
---  
### Hardware Architecture  
<img width="389" height="195" alt="Image" src="https://github.com/user-attachments/assets/a5c05f58-4e45-4e7f-b522-e4ccf926fada" />  

<img width="459" height="263" alt="Image" src="https://github.com/user-attachments/assets/64d7aed9-fb4f-4e66-b12f-aa4fd94c9669" />  

### Software Architecture  
<img width="361" height="196" alt="Image" src="https://github.com/user-attachments/assets/a076f86d-dd8b-4177-bc85-7a022777ff40" />  

### [Mission]  
|미션|설명|
|---|---|
|차선 주행|차량이 트랙을 2바퀴 주행하여 시간을 측정하고 패널티 시간을 합산하여 점수 산정|
|신호등 인식|신호등 신호에 맞는 동작을 보여야함|  
|장애물 회피|왼쪽과 오른쪽 차선에 각 4개의 slot이 있으며 장애물 하나가 2개의 slot을 차지하여 9가지의 경우의 수 존재|  
|수직 주차|T자 주차로 IN에서 출발하여 후진으로 수직 주차한 후 2초 이상 정차한 뒤에 빠져나와 출구로 진입하면 미션 성공|

### [Algorithm]  
1️⃣ 차선 주행 알고리즘  
<img width="261" height="229" alt="Image" src="https://github.com/user-attachments/assets/02ffbc6e-f1fa-47be-a743-e4b12fa9f268" />  
차선 인식을 위해 영상을 전처리하여, ROI 이미지를 추출한 뒤 top-down view로 전환한다.  
이후 영상을 이진화하고, 이진화된 영상을 바탕으로 개발한 차선 인식 알고리즘을 수행하여 차선의 위치를 검출한다.  
차선을 찾게 되면 2차 곡선을 그려 넣고, 찾지 못하면 Lane_not_found 모드로 동작하게 한다. 이후 Kalman Filter를 적용하여  
도로의 중점과 차량의 중점 사이의 거리를 구해 이를 바탕으로 PD 제어를 통해 조향각을 결정한다.  
