# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 구구단을 출력하는 프로그램
def main():
    for i in range(8):
        print(f"{i+2} 단 입니다.")
        for j in range(9):
            print(f"{i+2} X {j+1} = {(i+2)*(j+1)}")


if __name__ == "__main__":
    main()
