import time
import random

class Dice:    # 주사위 흔들기
    def __init__(self,zumi,screen):
        self.zumi = zumi
        self.screen = screen


    def roll_dice(self,sensitivity = 20, range = 6):
        # range : 주사위 눈 범위를 설정합니다.
        # sensitivity : 감도 값이 클수록 주미를 더 빠르게 흔들어야 합니다. 

        if(sensitivity > 120 or sensitivity < 1):
            print("Sensitivity input range is from 1 to 120")
            
        elif(range > 1000 or range < 1):         
            print("The dice input range is 1 to 1000.")
          
        else:
            # 주미의 자이로를 리셋합니다.(자이로 x축과 자이로 y축의 값을 0으로 설정)
            self.zumi.reset_gyro()

            # 현재 자이로 각도를 기록합니다.
            # 이 값은 이전 상태의 자이로 각도 값으로 사용됩니다.
            old_angleX = int(self.zumi.update_angles()[0])
            old_angleY = int(self.zumi.update_angles()[1])  

            shake = False # 흔들림을 기록하는 변수를 선언하고 값을 False로 입력합니다.

            # 흔들림 감지를 시작합니다.
            print("")
            print("- Shake the dice")
            message = "Shake the dice"
            self.screen.draw_text(message)   

            while True:

                # 자이로 각도 값을 읽어옵니다.    
                current_angleX = int(self.zumi.update_angles()[0])
                current_angleY = int(self.zumi.update_angles()[1])

                # 이전 상태의 자이로 각도와 현재의 자이로 각도의 차이를 구합니다.
                difference_angleX = old_angleX - current_angleX
                difference_angleY = old_angleY - current_angleY


                # 현재 상태의 자이로 각도 값을 이전 상태의 각도 값에 입력합니다. 
                old_angleX = current_angleX
                old_angleY = current_angleY        

                time.sleep(0.1)

                # 각도의 차이 값을 비교합니다. 
                # 차이값이 조건을 만족하는 경우 흔들림이 있다고 판단합니다.    
                # 각도 차이가 생기는 경우에 shake 값을 True로 바꾸고 While문을 빠져나가게 됩니다.    
                if difference_angleX > sensitivity or difference_angleX < -sensitivity or difference_angleY > sensitivity or difference_angleY < -sensitivity:
                    # 흔들림이 생길때까지 감지합니다.
                    if shake == False:
                        shake = True
                        message = "- Shake Detected"                
                        self.screen.draw_text(message)    

                # 흔들림이 생기고 멈출 때까지 감지합니다.
                else:
                    if shake == True:
                        # diceNumber변수에 랜덤 함수를 사용하여 1에서 range값 범위의 임의의 정수를 입력합니다.
                        diceNumber = random.randint(1, range)

                        # diceNumber 값을 출력합니다.
                        print("- Dice Number")
                        print(diceNumber)

                        message = "   Dice Number          "
                        self.screen.draw_text(message + str(diceNumber))   

                        # 루프를 빠져나옵니다.
                        break
                    
              
   