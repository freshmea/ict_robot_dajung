# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 파일을 읽어서 BMI를 계산하는 프로그램
def main():
    with open("data/info.txt", "r") as f:
        for line in f:
            (name, weight, height) = line.strip().split(", ")
            if (not name) or (not weight) or (not height):
                continue
            bmi = int(weight) / (int(height) / 100 * int(height) / 100)
            result = ""
            if 25 <= bmi:
                result = "과체중"
            elif 18.5 <= bmi:
                result = "정상체중"
            else:
                result = "저체중"

            # print(
            #     "\n".join(["이름: {}", "몸무게: {}", "키: {}", "BMI: {}", "결과: {}"]).format(
            #         name, weight, height, bmi, result
            #     )
            # )
            print(
                f"이름: {name}\n몸무게: {weight}\n키: {height}\nBMI: {bmi}\n결과: {result}\n\n"
            )


if __name__ == "__main__":
    main()
