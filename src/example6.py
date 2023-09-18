# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 리스트 내부에 있는 값 찾기
def main():
    numbers = [52, 273, 32, 103, 90, 10, 275]

    print("요소 내부에 있는 값 찾기")
    print(f"- {273} 는 index: {numbers.index(273)}")
    print()

    print("요소 내부에 있는 값 찾기")
    number = 10000
    try:
        print(f"- {number} 는 {numbers.index(number)}")
    except ValueError:
        print(f"-{number} 리스트 내부에 없는 값입니다.")
    print()

    print("---- 정상적으로 종료가 되었습니다.")


if __name__ == "__main__":
    main()
