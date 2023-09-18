# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 2진수로 변환했을 때 0이 하나만 포함된 숫자를 구하는 프로그램
def main():
    output = [i for i in range(100 + 1) if f"{i:b}".count("0") == 1]

    for i in output:
        print(f"{i} : {i:b}")
    print(f"합계: {sum(output)}")


if __name__ == "__main__":
    main()
