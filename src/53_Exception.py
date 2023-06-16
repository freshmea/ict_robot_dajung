import sys

def main():
    li = list(range(0,101,5)) # 100 까지 5의 배수
    while True:
        try:
            input_int = int(input('정수 입력(0 ~ 21 까지): '))
            print(f'이 리스트의 {input_int} 번째 요소는 {li[input_int]} 입니다.')
            break
        except ValueError as e:
            print(e)
            print('정수만 입력해 주세요.')
        except IndexError as e:
            print(e)
            print('0 ~ 21 까지만 입력해 주세요.')
        except KeyboardInterrupt:
            print ('키보드로 스탑')
            sys.exit()
        except:
            print('모든 에러')

if __name__ == '__main__':
    main()
