PI = 3.141592

def number_input():
    output = input('숫자입력: ')
    return float(output)

def get_circumference(radius):
    return 2*PI*radius

def get_circle_area(radius):
    return PI*radius*radius

def main():
    print('모듈의 __name__출력')
    print(__name__)
    print()

if __name__ == '__main__':
    main()